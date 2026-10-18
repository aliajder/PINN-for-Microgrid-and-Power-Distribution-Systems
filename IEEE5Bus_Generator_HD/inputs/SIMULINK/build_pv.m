function build_pv(mdl)
%% build_pv.m  (Task 5.7)
% Builds the PV subsystem (physical single-diode array + P&O MPPT + averaged
% boost DC-DC) inside the master model. Feeds the common DC bus (§3.6/§3.7).
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.7 (PV), CLAUDE.md A1/R4/R5
% Params    : DATA/pv_params.mat (struct `pv`, from build_pv_params.m, Rule 8)
% Physics   : PARAMS/pv_iv.m (diode, offline twin) — the diode below is the
%             INLINE copy (params injected as literals, ESS-style); the test
%             cross-checks the Simulink MPP against the offline pv_mpp sweep.
%
% WHAT THIS MODULE IS (hybrid, Task 5.7):
%   RAW irradiance G [W/m^2] + ambient T [°C] drive a physical single-diode
%   array; a Perturb & Observe MPPT finds the array's V_mpp; an averaged
%   (power-conserving) boost DC-DC injects that power into the 1500 V bus.
%   Renewables.ninja's processed `electricity` is NOT an input — validation only.
%
%      In_irr_Wm2 ─┐
%      In_T_amb_C ─┤  [pv_cell] I,P_panel ─► [mppt_po] V_ref ─► (V_op delay) ┐
%                  └───────────────────────────────────────────────────────┘ (loop)
%                     P_panel ─► [dcdc_ctrl] I_cmd ─►(delay)─► CCS ─► Elec_DC_out ─► DCBus/Elec_PV_term
%                                            (I = P_panel·η_dcdc·η_sys / V_dc)
%
% ── DESIGN DECISIONS (2026-06-20) ─────────────────────────────────────────
%  P1. SIGNAL-LEVEL diode + MPPT (mirrors ESS H1). The array I–V, the MPPT
%      operating point V_ref, and the boost are computed in MATLAB Functions;
%      the whole electrical PV footprint on the bus = ONE Controlled Current
%      Source, I = P_bus·1e6/V_dc (same CCS port map as ESS: RConn1=PS cmd,
%      LConn1=+rail, RConn2=gnd). No physical panel node at averaged scope (R5).
%  P2. INPORTS, not internal From Workspace. In_irr_Wm2 / In_T_amb_C are driven
%      externally — the test injects representative constants, the full model
%      wires From Workspace `pv_data` at master level (as the load profiles do).
%      Keeps the module isolated-testable (ESS In_Pref pattern).
%  P3. TWO algebraic-loop Unit Delays (ESS H6 lesson):
%        (a) V_ref → V_op delay breaks  pv_cell → mppt_po → V_ref → pv_cell.
%        (b) I_cmd → CCS delay breaks    V_dc → I=P/V_dc → CCS → network → V_dc.
%      S-PS converter on the CCS path keeps the τ=1e-4 filter for the DAE deriv.
%  P4. P&O MPPT perturbs every 1 ms (period/dt = 20 steps), ΔV = 2 V on a
%      ~650 V scale (0.3%) → small steady ripple. Clamp V_ref ∈ [50, V_oc_stc].
%  P5. Delivered P_bus = P_panel · η_dcdc · η_sys. η_sys (0.80, performance
%      ratio) is the non-thermal derate; temperature is in the diode (NOCT cell
%      heating + V_oc/I_sc temp-coeffs) — hot day → lower power, verified.

% ── Parameters (Rule 8: from DATA/pv_params.mat) ──────────────────────────
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
pv   = load(fullfile(root,'DATA','pv_params.mat')).pv;
dt_s = str2double(get_param(mdl,'FixedStep'));     % 50 µs

% ── Subsystem container ───────────────────────────────────────────────────
ssName = 'PV';  ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 520 360 660]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY PORTS
% =====================================================================
dcPort = [ssPath '/Elec_DC_out'];
add_block('built-in/PMIOPort', dcPort, 'Side', 'Left', 'Position', [40 300 60 320]);
add_block('built-in/Inport',  [ssPath '/In_irr_Wm2'], 'Position', [40 80 60 98]);
add_block('built-in/Inport',  [ssPath '/In_T_amb_C'], 'Position', [40 130 60 148]);
add_block('built-in/Outport', [ssPath '/Out_P_pv_MW'], 'Position', [1180 80 1200 98]);
add_block('built-in/Outport', [ssPath '/Out_V_pv_V'],  'Position', [1180 130 1200 148]);
add_block('built-in/Outport', [ssPath '/Out_I_pv_A'],  'Position', [1180 180 1200 198]);
add_block('built-in/Outport', [ssPath '/Out_irr_Wm2'], 'Position', [1180 230 1200 248]);
add_line(ssPath, 'In_irr_Wm2/1', 'Out_irr_Wm2/1', 'autorouting','on');   % P2 pass-through

% =====================================================================
%   V_dc SENSOR (parallel at the terminal — ESS H2 pattern)
% =====================================================================
add_block('ee_lib/Connectors & References/Electrical Reference', [ssPath '/Gnd'], 'Position', [340 460 380 490]);
vs = [ssPath '/V_dc_sens'];
add_block('ee_lib/Sensors & Transducers/Voltage Sensor', vs, 'Position', [180 360 240 420]);
add_line(ssPath, conn(dcPort,'RConn',1), conn(vs,'LConn',1));
add_line(ssPath, conn(vs,'RConn',2), conn([ssPath '/Gnd'],'LConn',1));
add_block('nesl_utility/PS-Simulink Converter', [ssPath '/Vdc_PS2S'], 'Position', [180 480 220 510]);
set_param([ssPath '/Vdc_PS2S'], 'Unit', 'V');
add_line(ssPath, conn(vs,'RConn',1), conn([ssPath '/Vdc_PS2S'],'LConn',1));

% =====================================================================
%   pv_cell — single-diode I–V (INLINE copy of pv_iv.m; params as literals)
% =====================================================================
cell = [ssPath '/pv_cell'];
add_block('simulink/User-Defined Functions/MATLAB Function', cell, 'Position', [320 70 480 170]);
set_mlfcn(cell, { ...
 'function [I_pv, P_panel_W] = pv_cell(V_op, G, T_amb)'
 '%#codegen'
 '% Single-diode array (INLINE twin of PARAMS/pv_iv.m). One equivalent array.'
 ['Voc_stc = ' num2str(pv.V_oc_stc) ';  N_s = ' num2str(pv.N_s) ';  nID = ' num2str(pv.n) ';']
 ['Isc_stc = ' num2str(pv.I_sc_stc,'%.6g') ';  aIsc = ' num2str(pv.alpha_Isc) ';  bVoc = ' num2str(pv.beta_Voc) ';  NOCT = ' num2str(pv.NOCT) ';']
 '    q = 1.602176634e-19;  kB = 1.380649e-23;'
 '    Tcell = T_amb + (NOCT - 20)/800 * G;'
 '    a  = N_s * nID * kB * (Tcell + 273.15) / q;'
 '    Iph = (G/1000) * Isc_stc * (1 + aIsc*(Tcell - 25));'
 '    if Iph <= 0'
 '        I_pv = 0; P_panel_W = 0; return;'
 '    end'
 '    Voc_T = Voc_stc * (1 + bVoc*(Tcell - 25));'
 '    I0 = Iph / (exp(Voc_T/a) - 1);'
 '    I_pv = Iph - I0*(exp(V_op/a) - 1);'
 '    if I_pv < 0, I_pv = 0; end'
 '    P_panel_W = V_op * I_pv;'
 'end'});
setDiscrete(cell, dt_s);
add_line(ssPath, 'In_irr_Wm2/1', 'pv_cell/2', 'autorouting','on');
add_line(ssPath, 'In_T_amb_C/1', 'pv_cell/3', 'autorouting','on');

% =====================================================================
%   mppt_po — Perturb & Observe (P4); outputs V_ref
% =====================================================================
mppt = [ssPath '/mppt_po'];
add_block('simulink/User-Defined Functions/MATLAB Function', mppt, 'Position', [560 70 700 150]);
set_mlfcn(mppt, { ...
 'function V_ref = mppt_po(P_meas, dt)'
 '%#codegen'
 '% Perturb & Observe MPPT. Perturbs every ~1 ms; climbs the P(V) hill.'
 ['Vinit = ' num2str(round(0.78*pv.V_oc_stc)) ';  dV = 2;  Vmin = 50;  Vmax = ' num2str(pv.V_oc_stc) ';  period = 1e-3;']
 '    persistent Vr Pp dir cnt'
 '    if isempty(Vr), Vr = Vinit; Pp = 0; dir = 1; cnt = 0; end'
 '    nper = round(period/dt);'
 '    cnt = cnt + 1;'
 '    if cnt >= nper'
 '        if P_meas < Pp, dir = -dir; end     % got worse → reverse direction'
 '        Vr = Vr + dir*dV;'
 '        if Vr > Vmax, Vr = Vmax; end'
 '        if Vr < Vmin, Vr = Vmin; end'
 '        Pp = P_meas;'
 '        cnt = 0;'
 '    end'
 '    V_ref = Vr;'
 'end'});
setDiscrete(mppt, dt_s);
add_block('built-in/Constant', [ssPath '/dt_m'], 'Value', num2str(dt_s), 'Position', [480 200 530 220]);
add_line(ssPath, 'pv_cell/2', 'mppt_po/1', 'autorouting','on');   % P_panel_W → mppt
add_line(ssPath, 'dt_m/1',    'mppt_po/2', 'autorouting','on');

% V_ref → Unit Delay → V_op (breaks the pv_cell↔mppt algebraic loop, P3a)
add_block('simulink/Discrete/Unit Delay', [ssPath '/Vop_delay'], ...
    'SampleTime', num2str(dt_s), 'InitialCondition', num2str(round(0.78*pv.V_oc_stc)), ...
    'Position', [560 30 590 50]);
add_line(ssPath, 'mppt_po/1',  'Vop_delay/1', 'autorouting','on');
add_line(ssPath, 'Vop_delay/1','pv_cell/1',   'autorouting','on');   % V_op → pv_cell
add_line(ssPath, 'Vop_delay/1','Out_V_pv_V/1', 'autorouting','on');  % operating voltage
add_line(ssPath, 'pv_cell/1',  'Out_I_pv_A/1', 'autorouting','on');  % I at V_op

% =====================================================================
%   dcdc_ctrl — averaged power-conserving boost → bus current command
% =====================================================================
dcdc = [ssPath '/dcdc_ctrl'];
add_block('simulink/User-Defined Functions/MATLAB Function', dcdc, 'Position', [560 260 720 340]);
set_mlfcn(dcdc, { ...
 'function [I_cmd_A, P_bus_MW] = dcdc_ctrl(P_panel_W, V_dc_V)'
 '%#codegen'
 '% Averaged boost: deliver P_panel·η to the bus as a current I = P_bus/V_dc.'
 ['eta = ' num2str(pv.eta_dcdc*pv.eta_sys,'%.6g') ';  Vfloor = ' num2str(pv.V_floor) ';']
 '    P_bus_W  = P_panel_W * eta;'
 '    I_cmd_A  = P_bus_W / max(V_dc_V, Vfloor);'
 '    P_bus_MW = P_bus_W / 1e6;'
 'end'});
setDiscrete(dcdc, dt_s);
add_line(ssPath, 'pv_cell/2',  'dcdc_ctrl/1', 'autorouting','on');   % P_panel_W
add_line(ssPath, 'Vdc_PS2S/1', 'dcdc_ctrl/2', 'autorouting','on');   % V_dc_meas
add_line(ssPath, 'dcdc_ctrl/2','Out_P_pv_MW/1', 'autorouting','on');

% =====================================================================
%   CONTROLLED CURRENT SOURCE on the + rail (P1; ESS CCS pattern)
%   I_cmd → Unit Delay (P3b) → S-PS (filter) → CCS RConn1.
% =====================================================================
add_block('simulink/Discrete/Unit Delay', [ssPath '/I_delay'], ...
    'SampleTime', num2str(dt_s), 'Position', [760 285 790 315]);
add_block('nesl_utility/Simulink-PS Converter', [ssPath '/I_S2PS'], 'Position', [820 285 860 315]);
set_param([ssPath '/I_S2PS'], 'Unit', 'A', 'FilteringAndDerivatives', 'filter', ...
    'SimscapeFilterOrder', '1', 'InputFilterTimeConstant', '1e-4');
add_line(ssPath, 'dcdc_ctrl/1', 'I_delay/1', 'autorouting','on');
add_line(ssPath, 'I_delay/1',   'I_S2PS/1',  'autorouting','on');

ccs = [ssPath '/PV_CCS'];
add_block('fl_lib/Electrical/Electrical Sources/Controlled Current Source', ccs, 'Position', [520 360 580 440]);
add_line(ssPath, conn([ssPath '/I_S2PS'],'RConn',1), conn(ccs,'RConn',1));   % PS cmd
add_line(ssPath, conn(ccs,'LConn',1), conn(dcPort,'RConn',1));               % + rail
add_line(ssPath, conn(ccs,'RConn',2), conn([ssPath '/Gnd'],'LConn',1));      % return (F1)

fprintf('  build_pv: 80 MW single-diode array + P&O MPPT + averaged boost (eta=%.2f); feeds Elec_PV_term.\n', ...
    pv.eta_dcdc*pv.eta_sys);
end % build_pv


% =====================================================================
%                              HELPERS
% =====================================================================
function h = conn(blkPath, side, idx)
ph = get_param(blkPath, 'PortHandles');
h  = ph.(side)(idx);
end

function set_mlfcn(blkPath, lines)
cfg = get_param(blkPath, 'MATLABFunctionConfiguration');
cfg.FunctionScript = strjoin(lines, newline);
end

function setDiscrete(blkPath, dt_s)
cfg = get_param(blkPath, 'MATLABFunctionConfiguration');
cfg.UpdateMethod = 'Discrete';
cfg.SampleTime   = num2str(dt_s);
end
