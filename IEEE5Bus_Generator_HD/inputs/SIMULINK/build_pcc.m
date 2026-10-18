function build_pcc(mdl)
%% build_pcc.m
% Builds the PCC (Point of Common Coupling) subsystem inside the master model.
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.3 (PCC)
% Master    : SIMULINK/build_microgrid_eelib.m calls this with the model name
% Params    : f0 and FixedStep read from base workspace / model (Rule 8/10)
%
% WHAT THIS MODULE IS (per IO_CONTRACTS §3.3):
%   Step-up/down transformer 230/33 kV (300 MVA) + main breaker CB1 +
%   frequency / voltage / power measurement at the 33 kV (MV) side.
%
%   TransmissionNetwork ──~230kV~──►│ Elec_HV_230kV
%                                   │   [XFMR 230/33 kV, Δ-Yg, 300 MVA]
%                                   │        │ MV 33 kV
%                                   │     [CB1 breaker] ── (RC snubber ∥)
%                                   │        │
%                                   │   [I sensor]──[V sensor]──► Elec_MV_33kV ──► (MG AC bus, later)
%                                   │        │
%                                   │   PLL→f_meas(Goto)  V_to_pu→V_pcc(Goto)  P/Q→ports
%
% ── DESIGN DECISIONS taken while building (2026-06-06) ────────────────────
%
%  D1. NATIVE three-phase breaker.  IO_CONTRACTS §3.3 assumed "ee_lib has no
%      generic Three-Phase Breaker → build a 3× switch composite". That is
%      OUTDATED: ee_lib SHIPS `Circuit Breaker (Three-Phase)`. We use it.
%      Verified port map (probed 2026-06-06):
%        LConn(1) = PS control signal  (NOT 3-phase!)
%        LConn(2) = 3-phase electrical IN
%        RConn(1) = 3-phase electrical OUT
%      Control semantics (threshold = 0.5, breaker_behavior=zeroCrossingEnable):
%        control > 0.5 → OPEN (trip) ;  control < 0.5 → CLOSED.
%      Our project convention (IO_CONTRACTS §1.1) is CB_PCC: 1=closed, 0=open.
%      So we feed the breaker the INVERTED command  ctrl = 1 - In_CB_PCC_cmd.
%      zeroCrossingEnable opens at the next current zero crossing → it already
%      suppresses the L·di/dt interruption spike (so the RC snubber below is a
%      belt-and-suspenders placeholder, not the primary mitigation).
%
%  D2. VECTOR GROUP Δ(HV)-Yg(MV).  CLAUDE.md A3 reads "33/230 kV, Δ-Y" (33 kV=Δ)
%      but IO_CONTRACTS §3.3 reads "230/33 kV, Δ-Y" (230 kV=Δ). The two docs
%      conflict. We choose **HV 230 kV = Δ (delta11), MV 33 kV = Yg** because:
%        • the 33 kV MG bus needs a solid ground reference for phase-voltage
%          measurement, the PLL, and a defined common mode BEFORE the
%          grid-forming VSC exists — a delta MV side would float;
%        • the HV side does not need a transformer ground (the TransmissionNetwork
%          already provides one via the grounded slack / loads);
%        • this is the textbook substation step-down group (Dyn11).
%      ⚠ FLAGGED for advisor: if A3's literal "33 kV = Δ" is mandatory, the MG
%      AC bus must instead supply its own MV ground reference (grounding XFMR).
%      A 30° Δ-Y phase shift between HV and MV is physical and expected.
%
%  D3. MEASUREMENTS on the MG-facing node (downstream of CB1), per §3.3. When
%      CB1 opens, V_pcc / f_meas reflect the (now islanded) MG node — which,
%      until the VSC is built, collapses to ~0. That is the correct incomplete-
%      model behaviour and is exactly what the standalone open/closed test checks.

% ── Parameters (no magic numbers; units in names) ────────────────────────
f0_Hz       = evalin('base','f0');                 % 50 Hz (Rule 10)
dt_s        = str2double(get_param(mdl,'FixedStep')); % solver step for the PLL

S_xfmr_VA   = 300e6;     % transformer rating              [VA]   (A3)
V_hv_LL     = 230e3;     % HV winding line-line RMS         [V]
V_mv_LL     = 33e3;      % MV winding line-line RMS         [V]
Xleak_pu    = 0.10;      % total leakage reactance         [pu]   (~10%, A3)
Rw_pu       = 0.002;     % per-winding resistance          [pu]   (X/R≈25)

R_snub_ohm  = 200;       % breaker RC snubber resistance    [Ω]  % TUNED: placeholder, tune in Template B (islanding)
C_snub_F    = 1e-6;      % breaker RC snubber capacitance    [F]  % TUNED: placeholder, tune in Template B (islanding)

% SRF-PLL gains (placeholders; tune if HIL exposes jitter). Loop ≈ tens of ms.
PLL_Kp      = 180;       % proportional [rad/s per pu-q]
PLL_Ki      = 3200;      % integral     [rad/s² per pu-q]

% ── Subsystem container ──────────────────────────────────────────────────
ssName = 'PCC';
ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 420 360 540]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY PORTS
% =====================================================================
% Electrical: HV (to TransmissionNetwork) on the left, MV (to MG) on the right.
hvPort = [ssPath '/Elec_HV_230kV'];
add_block('built-in/PMIOPort', hvPort, 'Side', 'Left',  'Position', [40 300 60 320]);
mvPort = [ssPath '/Elec_MV_33kV'];
add_block('built-in/PMIOPort', mvPort, 'Side', 'Right', 'Position', [1500 300 1520 320]);

% Control input (breaker command) and power-flow outputs.
add_block('built-in/Inport',  [ssPath '/In_CB_PCC_cmd'], 'Position', [40 120 60 138]);
add_block('built-in/Outport', [ssPath '/Out_P_pcc_MW'],  'Position', [1500 120 1520 138]);
add_block('built-in/Outport', [ssPath '/Out_Q_pcc_MVAr'],'Position', [1500 170 1520 188]);

% =====================================================================
%   TRANSFORMER  (HV 230 kV Δ11  /  MV 33 kV Yg, 300 MVA, 10% leakage)
% =====================================================================
xf = [ssPath '/XFMR_230_33'];
add_block('ee_lib/Passive/Transformers/Two-Winding Transformer (Three-Phase)', xf, ...
    'Position', [180 280 280 360]);
set_param(xf, ...
    'SRated',  num2str(S_xfmr_VA), ...
    'FRated',  num2str(f0_Hz), ...
    'Winding1Connection', 'ee.enum.windingconnection.delta11', ... % HV = Δ (D2)
    'VRated1', num2str(V_hv_LL), ...
    'Winding2Connection', 'ee.enum.windingconnection.Yg', ...      % MV = Yg (D2)
    'VRated2', num2str(V_mv_LL), ...
    'leakage_reactance_option', 'ee.enum.transformer_leakage.include', ...
    'pu_Xl1',  num2str(Xleak_pu/2), 'pu_Xl2', num2str(Xleak_pu/2), ...
    'pu_Rw1',  num2str(Rw_pu),      'pu_Rw2', num2str(Rw_pu));

% HV winding (LConn) ↔ HV boundary port
add_line(ssPath, conn(hvPort,'RConn',1), conn(xf,'LConn',1));

% =====================================================================
%   BREAKER CB1  (native Circuit Breaker (Three-Phase), inverted command)
% =====================================================================
cb = [ssPath '/CB1'];
add_block('ee_lib/Switches & Breakers/Circuit Breaker (Three-Phase)', cb, ...
    'Position', [420 280 480 360]);
set_param(cb, ...
    'breaker_behavior', 'ee.enum.switches.breakerBehavior.zeroCrossingEnable', ...
    'threshold', '0.5', 'R_closed', '1e-3', 'G_open', '1e-6');

% Transformer MV (RConn) → breaker electrical IN (LConn(2))
add_line(ssPath, conn(xf,'RConn',1), conn(cb,'LConn',2));

% --- Breaker command path:  ctrl = 1 - In_CB_PCC_cmd  (D1 inversion) ---
add_block('built-in/Constant', [ssPath '/One'], 'Value', '1', 'Position', [120 60 150 90]);
add_block('simulink/Math Operations/Sum', [ssPath '/Inv_cmd'], ...
    'Inputs', '+-', 'Position', [200 70 230 100]);   % One - cmd
add_block('nesl_utility/Simulink-PS Converter', [ssPath '/cmd_S2PS'], ...
    'Position', [300 75 340 105]);
set_param([ssPath '/cmd_S2PS'], 'FilteringAndDerivatives', 'filter', ...
    'SimscapeFilterOrder', '1', 'InputFilterTimeConstant', '1e-4');
add_line(ssPath, 'One/1',           'Inv_cmd/1', 'autorouting','on');
add_line(ssPath, 'In_CB_PCC_cmd/1', 'Inv_cmd/2', 'autorouting','on');
add_line(ssPath, 'Inv_cmd/1',       'cmd_S2PS/1','autorouting','on');
add_line(ssPath, conn([ssPath '/cmd_S2PS'],'RConn',1), conn(cb,'LConn',1));  % PS → control

% --- Breaker status mirror → Goto CB_PCC (global) ---
add_block('simulink/Signal Routing/Goto', [ssPath '/Goto_CB_PCC'], ...
    'GotoTag', 'CB_PCC', 'TagVisibility', 'global', 'Position', [300 120 360 138]);
add_line(ssPath, 'In_CB_PCC_cmd/1', 'Goto_CB_PCC/1', 'autorouting','on');

% =====================================================================
%   RC SNUBBER across the breaker poles (placeholder, §3.3 / D1)
% =====================================================================
snub = [ssPath '/CB1_Snubber'];
add_block('ee_lib/Passive/RLC Assemblies/RLC (Three-Phase)', snub, ...
    'Position', [420 420 480 480]);
set_param(snub, 'component_structure', 'ee.enum.rlc.structure.SeriesRC', ...
    'R', num2str(R_snub_ohm), 'C', num2str(C_snub_F));
% In parallel with CB1: snubber LConn at breaker-IN node, RConn at breaker-OUT node
add_line(ssPath, conn(xf,'RConn',1), conn(snub,'LConn',1));   % branch off CB IN node
add_line(ssPath, conn(cb,'RConn',1), conn(snub,'RConn',1));   % branch off CB OUT node

% =====================================================================
%   CURRENT SENSOR (series) — breaker OUT → MV measurement node
%   + = current flowing transmission → MG (importing), per §1.1 P_pcc sign.
% =====================================================================
isens = [ssPath '/I_pcc'];
add_block('ee_lib/Sensors & Transducers/Current Sensor (Three-Phase)', isens, ...
    'Position', [560 280 620 360]);
add_line(ssPath, conn(cb,'RConn',1),    conn(isens,'LConn',1));   % CB out → I in
add_line(ssPath, conn(isens,'RConn',2), conn(mvPort,'RConn',1));  % I out → MV port (MV node; PMIOPort exposes RConn internally regardless of Side)

% =====================================================================
%   VOLTAGE SENSOR (parallel) at the MV measurement node
% =====================================================================
vsens = [ssPath '/V_pcc'];
add_block('ee_lib/Sensors & Transducers/Phase Voltage Sensor (Three-Phase)', vsens, ...
    'Position', [700 380 760 450]);
add_block('ee_lib/Connectors & References/Grounded Neutral (Three-Phase)', [ssPath '/V_pcc_ref'], ...
    'Position', [700 500 760 530]);
add_line(ssPath, conn(isens,'RConn',2), conn(vsens,'LConn',1));        % MV node → V sensor
add_line(ssPath, conn(vsens,'RConn',2), conn([ssPath '/V_pcc_ref'],'LConn',1));

% =====================================================================
%   PS → Simulink bridges for the sensor signals (3-phase each)
% =====================================================================
add_block('nesl_utility/PS-Simulink Converter', [ssPath '/I_PS2S'], 'Position', [660 200 700 230]);
add_block('nesl_utility/PS-Simulink Converter', [ssPath '/V_PS2S'], 'Position', [800 395 840 425]);
add_line(ssPath, conn(isens,'RConn',1), conn([ssPath '/I_PS2S'],'LConn',1));
add_line(ssPath, conn(vsens,'RConn',1), conn([ssPath '/V_PS2S'],'LConn',1));

% =====================================================================
%   MEASUREMENT MATH (Simulink side)
% =====================================================================
% (a) PLL → f_meas (Goto, global). atan2/SRF-PLL with persistent state.
pll = [ssPath '/PLL_freq'];
add_block('simulink/User-Defined Functions/MATLAB Function', pll, 'Position', [900 300 1020 360]);
set_mlfcn(pll, { ...
 'function f_meas_Hz = PLL_freq(v_abc, dt_s, f0_Hz, Kp, Ki)'
 '%#codegen'
 '% Synchronous-reference-frame PLL on the MV phase voltages.'
 '% Clarke (amplitude-invariant) → normalise → q-axis error → PI → θ.'
 '% Persistent θ and integral are discrete states (HIL/codegen friendly).'
 '    persistent th vi'
 '    if isempty(th), th = 0; vi = 0; end'
 '    va = v_abc(1); vb = v_abc(2); vc = v_abc(3);'
 '    valpha = (2/3)*(va - 0.5*vb - 0.5*vc);'
 '    vbeta  = (1/sqrt(3))*(vb - vc);'
 '    Vm = hypot(valpha, vbeta) + 1e-9;       % avoid /0 at start-up'
 '    sa = valpha/Vm; sb = vbeta/Vm;          % unit phasor (amplitude-independent)'
 '    vq = -sa*sin(th) + sb*cos(th);          % q-axis error → 0 when locked'
 '    vi = vi + Ki*vq*dt_s;                    % PI integral'
 '    w  = 2*pi*f0_Hz + Kp*vq + vi;            % estimated angular frequency'
 '    th = mod(th + w*dt_s, 2*pi);             % advance & wrap phase'
 '    f_meas_Hz = w/(2*pi);'
 'end'});
% The PLL holds discrete states (persistent th, vi) → it MUST run on a
% discrete sample time, not the inherited/continuous default (otherwise
% "persistent variables invalid on continuous sample time"). Run it at the
% solver step so the internal dt_s = 50 µs matches the actual update rate.
cfgPLL = get_param(pll, 'MATLABFunctionConfiguration');
cfgPLL.UpdateMethod = 'Discrete';
cfgPLL.SampleTime   = num2str(dt_s);
add_block('built-in/Constant', [ssPath '/dt_c'], 'Value', num2str(dt_s), 'Position', [800 250 850 270]);
add_block('built-in/Constant', [ssPath '/f0_c'], 'Value', num2str(f0_Hz),'Position', [800 280 850 300]);
add_block('built-in/Constant', [ssPath '/Kp_c'], 'Value', num2str(PLL_Kp),'Position',[800 310 850 330]);
add_block('built-in/Constant', [ssPath '/Ki_c'], 'Value', num2str(PLL_Ki),'Position',[800 340 850 360]);
add_line(ssPath, 'V_PS2S/1', 'PLL_freq/1', 'autorouting','on');
add_line(ssPath, 'dt_c/1',   'PLL_freq/2', 'autorouting','on');
add_line(ssPath, 'f0_c/1',   'PLL_freq/3', 'autorouting','on');
add_line(ssPath, 'Kp_c/1',   'PLL_freq/4', 'autorouting','on');
add_line(ssPath, 'Ki_c/1',   'PLL_freq/5', 'autorouting','on');
add_block('simulink/Signal Routing/Goto', [ssPath '/Goto_f_meas'], ...
    'GotoTag', 'f_meas', 'TagVisibility', 'global', 'Position', [1080 322 1140 340]);
add_line(ssPath, 'PLL_freq/1', 'Goto_f_meas/1', 'autorouting','on');

% (b) V_to_pu → V_pcc (Goto, global). LL-RMS pu on the 33 kV base.
vtopu = [ssPath '/V_to_pu'];
add_block('simulink/User-Defined Functions/MATLAB Function', vtopu, 'Position', [900 400 1020 460]);
set_mlfcn(vtopu, { ...
 'function V_pcc_pu = V_to_pu(v_abc, Vbase_LL)'
 '%#codegen'
 '% Line-to-line RMS per-unit from instantaneous phase voltages.'
 '% Amplitude-invariant Clarke magnitude = peak phase amplitude;'
 '% LL_RMS = peak_phase * sqrt(3)/sqrt(2)  →  pu = LL_RMS / Vbase_LL.'
 '    va = v_abc(1); vb = v_abc(2); vc = v_abc(3);'
 '    valpha = (2/3)*(va - 0.5*vb - 0.5*vc);'
 '    vbeta  = (1/sqrt(3))*(vb - vc);'
 '    Vphase_peak = hypot(valpha, vbeta);'
 '    V_pcc_pu = Vphase_peak * sqrt(3/2) / Vbase_LL;'
 'end'});
add_block('built-in/Constant', [ssPath '/Vbase_c'], 'Value', num2str(V_mv_LL), 'Position', [800 460 860 480]);
add_line(ssPath, 'V_PS2S/1',  'V_to_pu/1', 'autorouting','on');
add_line(ssPath, 'Vbase_c/1', 'V_to_pu/2', 'autorouting','on');
add_block('simulink/Signal Routing/Goto', [ssPath '/Goto_V_pcc'], ...
    'GotoTag', 'V_pcc', 'TagVisibility', 'global', 'Position', [1080 422 1140 440]);
add_line(ssPath, 'V_to_pu/1', 'Goto_V_pcc/1', 'autorouting','on');

% (c) P/Q calc → Out_P_pcc_MW, Out_Q_pcc_MVAr (direct output ports).
pq = [ssPath '/PQ_calc'];
add_block('simulink/User-Defined Functions/MATLAB Function', pq, 'Position', [900 130 1020 210]);
set_mlfcn(pq, { ...
 'function [P_MW, Q_MVAr] = PQ_calc(v_abc, i_abc)'
 '%#codegen'
 '% Instantaneous three-phase active & reactive power at the MV node.'
 '% Amplitude-invariant Clarke → P = 1.5(vα iα + vβ iβ),  Q = 1.5(vβ iα − vα iβ).'
 '% Sign: + current is transmission→MG, so +P = MG importing (§1.1).'
 '    va = v_abc(1); vb = v_abc(2); vc = v_abc(3);'
 '    ia = i_abc(1); ib = i_abc(2); ic = i_abc(3);'
 '    valpha = (2/3)*(va - 0.5*vb - 0.5*vc);  vbeta = (1/sqrt(3))*(vb - vc);'
 '    ialpha = (2/3)*(ia - 0.5*ib - 0.5*ic);  ibeta = (1/sqrt(3))*(ib - ic);'
 '    P_W   = 1.5*(valpha*ialpha + vbeta*ibeta);'
 '    Q_VAR = 1.5*(vbeta*ialpha - valpha*ibeta);'
 '    P_MW   = P_W   / 1e6;'
 '    Q_MVAr = Q_VAR / 1e6;'
 'end'});
add_line(ssPath, 'V_PS2S/1', 'PQ_calc/1', 'autorouting','on');
add_line(ssPath, 'I_PS2S/1', 'PQ_calc/2', 'autorouting','on');
add_line(ssPath, 'PQ_calc/1', 'Out_P_pcc_MW/1',  'autorouting','on');
add_line(ssPath, 'PQ_calc/2', 'Out_Q_pcc_MVAr/1','autorouting','on');

fprintf('  build_pcc: XFMR 230/33 Δ-Yg 300 MVA + native CB1 + PLL/V/P/Q. Goto: f_meas,V_pcc,CB_PCC.\n');
end % build_pcc


% =====================================================================
%                              HELPERS
% =====================================================================
function h = conn(blkPath, side, idx)
% Conserving/PS port handle by side ('LConn'/'RConn') and index.
ph = get_param(blkPath, 'PortHandles');
h  = ph.(side)(idx);
end

function set_mlfcn(blkPath, lines)
% Set a MATLAB Function block body via the modern configuration API.
% NOTE: never route through sprintf — '%' chars (e.g. %#codegen) would be
% interpreted as format specifiers and mangled.
cfg = get_param(blkPath, 'MATLABFunctionConfiguration');
cfg.FunctionScript = strjoin(lines, newline);
end
