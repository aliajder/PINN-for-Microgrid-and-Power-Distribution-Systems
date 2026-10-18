function build_ess(mdl)
%% build_ess.m
% Builds the battery ESS subsystem (battery + bidirectional DC-DC, averaged)
% inside the master model. THE OWNER OF DC-BUS VOLTAGE REGULATION (§3.9).
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.9 (ESS) + CLAUDE.md A1/R1/R2/R4
% Master    : SIMULINK/build_microgrid_eelib.m calls this with the model name
% Params    : mg.ess.* read from base workspace (Rule 8 — no hardcoded data)
%
% WHAT THIS MODULE IS (per IO_CONTRACTS §3.9):
%   80 MW / 200 MWh battery + averaged bidirectional DC-DC whose job is to
%   hold V_dc = 1500 V. EMS dispatch (In_Pref_MW) enters as feed-forward;
%   the V_dc PI supplies the fast trim. Replaces the TEST-ONLY ideal DC
%   source used in build_vsc_test.m.
%
%      DCBus/Elec_ESS_term ──~1500V~──►│ Elec_DC_io
%                                      │   [V_dc sensor ∥]──► ess_ctrl
%                                      │   [CCS: I = P_ess/V_dc] ← ess_ctrl
%                                      │       (+ rail; − = gnd, F1)
%                                      │   ess_ctrl: slew(Pref) + unslewed
%                                      │     PI(1500−V_dc) trim (H7) →
%                                      │     TOTAL clamp (Rule 4, SOC) → I_cmd
%                                      │   battery (signal-level Thevenin):
%                                      │     OCV(SOC) − R_int·I_batt, SOC ∫
%
% ── DESIGN DECISIONS taken while building (2026-06-10) ────────────────────
%
%  H1. SIGNAL-LEVEL THEVENIN BATTERY. The battery (OCV(SOC) + R_int) is
%      computed inside ess_ctrl, NOT built as a second electrical circuit.
%      A physical battery loop would be an isolated Simscape network →
%      would demand its own Solver Configuration and buy nothing at the
%      averaged scope (R5): the DC-DC decouples battery volts from bus
%      volts anyway; what matters is P, SOC, and the V_batt diagnostic.
%      Electrical footprint of the whole ESS = ONE Controlled Current
%      Source on the + rail — the same pattern §3.7/§3.8 prescribe for
%      PV/Wind. (Electrochemical battery model = paper-phase upgrade.)
%
%  H2. SELF-MEASURED V_dc. §3.9 lists an In_Vdc_meas_V port fed from
%      DCBus/Out_V_dc_V; as built the ESS senses V_dc with its OWN parallel
%      sensor at Elec_DC_io (same node — DCBus is a star). Self-containment
%      beats a top-level measurement wire; contract updated in v1.7.
%
%  H3. V_dc PI GAINS vs §3.9 (R2 stacking, justified):
%      target: V_dc loop FASTER than the VSC droop power dynamics (~60
%      rad/s poles, build_vsc G5) so the VSC sees a stiff bus, but SLOWER
%      than the 50 µs sampling so the 1-sample loop delay stays harmless.
%        Kp = 0.2 MW/V  → crossover ωc ≈ Kp/(C_dc·V0) = 2e5/(0.03·1500)
%                         ≈ 4.4 krad/s: ×70 above droop, ×14 below Nyquist
%                         (62.8 krad/s); delay phase cost ωc·dt ≈ 12.6°.
%        Ki = 5 MW/V/s  → PI zero at Ki/Kp = 25 rad/s ≪ ωc (§3.9 value).
%      §3.9's Kp=0.5 MW/V would put ωc at 11 krad/s — with the 1-sample
%      delay that rings at 50 µs sampling → reduced to 0.2 (% TUNED).
%      R2 margin: VSC behaves as a CPL of |Z_cpl| = V²/P ≥ 1500²/80 MW
%      ≈ 28 mΩ; ESS closed-loop output impedance ≈ V0/Kp = 7.5 mΩ < 28 mΩ
%      → negative-impedance criterion met with ≈3.7× margin.
%
%  H4. CONTROL LAW ORDER (§3.9, amended by H7): P_sum = slew(Pref FF) +
%      PI trim → TOTAL clamp (Rule 4): ±Pmax AND SOC-aware directional
%      zeroing (no discharge at SOC_min, no charge at SOC_max). Rule 7
%      anti-windup: integrator frozen while the previous step saturated.
%
%  H7. SLEW LIMIT ON THE DISPATCH FF ONLY — NOT on the V_dc PI trim.
%      First build applied the §3.9 slew (80 MW/s) to the SUMMED command:
%      80 MW/s = 4 kW per 50 µs step, so the 4.4 krad/s V_dc loop (H3)
%      could not actuate — describing-function lag ≈ 90° → limit cycle →
%      simulation diverged at 6–8 ms (SM Pe_LPF non-finite). Verified by
%      experiment: stable with an ideal source pinning V_dc (PI quiescent),
%      diverges the moment the ESS owns the rail. The 80 MW/s ramp models
%      EMS dispatch (Rule 6 inverter ramp class); the averaged DC-DC has
%      no sub-ms power slew limit, and Rule 4's total clamp still bounds
%      |P| ≤ 80 MW. So: slew(Pref) + unslewed PI trim → total clamp.
%
%  H5. EFFICIENCY BOOKKEEPING (η = 0.95 each way, mg.ess): the battery
%      side sees P_batt = P_ess/η when discharging (cells supply more than
%      the bus receives) and P_batt = P_ess·η when charging (cells store
%      less than the bus delivers). SOC integrates P_batt, not P_ess.
%
%  H6. Same algebraic-loop break as the VSC (memory lesson): sensor→ctrl→
%      I_cmd→network→sensor is algebraic; a Unit Delay (1 sample) breaks
%      it; the filtered S-PS converter (τ=1e-4) supplies the derivative.
%
% Verified port map (probed 2026-06-10):
%   Controlled Current Source (fl_lib/Electrical/Electrical Sources/...):
%     RConn(1)=PS command [A]; LConn(1) & RConn(2) = electrical terminals;
%     POSITIVE command INJECTS current OUT of LConn(1) (measured +1000 V
%     on 10 Ω @ +100 A) → LConn1 = + rail, RConn2 = return/ground.

% ── Parameters (no magic numbers; units in names) ─────────────────────────
ess        = evalin('base','mg.ess');                % Rule 8: from params
dt_s       = str2double(get_param(mdl,'FixedStep')); % 50 µs ctrl sample time

V_dc_set_V   = 1500;                  % DC bus setpoint               [V]  (A1)
E_ess_MWh    = ess.Emax_MWh;          % 200 MWh
P_ess_max_MW = ess.Pmax_MW;           % 80 MW (R4: cannot cover 300 MW alone)
SOC_init     = ess.SOC_init;          % 0.60 (params value; prompt said ~0.5 — both mid-range)
SOC_min      = ess.SOC_min;           % 0.15 (Rule 7 / quick-reference bounds)
SOC_max      = ess.SOC_max;           % 0.90
eta          = ess.eta_discharge;     % 0.95 each way (H5)

Kp_dc_MW_V   = 0.2;    % V_dc PI proportional  [MW/V]   % TUNED: H3 (§3.9 said 0.5 — rings at 50 µs)
Ki_dc_MW_Vs  = 5;      % V_dc PI integral      [MW/V/s] (§3.9)
slew_MW_s    = 80;     % inverter slew         [MW/s]   (§3.9)

V_oc_min_V   = 700;    % aggregate OCV at SOC=0          [V] % TUNED: placeholder Li-ion string
V_oc_max_V   = 900;    % aggregate OCV at SOC=1          [V] % TUNED: placeholder
R_int_ohm    = 1.6e-4; % aggregate internal resistance   [Ω] % TUNED: ~2% drop at 80 MW

% ── Subsystem container ───────────────────────────────────────────────────
ssName = 'ESS';
ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 1220 360 1360]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY PORTS
% =====================================================================
dcPort = [ssPath '/Elec_DC_io'];
add_block('built-in/PMIOPort', dcPort, 'Side', 'Left', 'Position', [40 300 60 320]);

add_block('built-in/Inport',  [ssPath '/In_Pref_MW'],    'Position', [40 80 60 98]);
add_block('built-in/Outport', [ssPath '/Out_SOC_ess'],   'Position', [1100 80 1120 98]);
add_block('built-in/Outport', [ssPath '/Out_P_ess_MW'],  'Position', [1100 130 1120 148]);
add_block('built-in/Outport', [ssPath '/Out_V_batt_V'],  'Position', [1100 180 1120 198]);

% =====================================================================
%   V_dc SENSOR (parallel at the terminal — H2)
%   Verified: LConn1=+, RConn1=PS out, RConn2=−.
% =====================================================================
add_block('ee_lib/Connectors & References/Electrical Reference', [ssPath '/Gnd'], ...
    'Position', [340 460 380 490]);
vs = [ssPath '/V_dc_sens'];
add_block('ee_lib/Sensors & Transducers/Voltage Sensor', vs, 'Position', [180 360 240 420]);
add_line(ssPath, conn(dcPort,'RConn',1), conn(vs,'LConn',1));
add_line(ssPath, conn(vs,'RConn',2), conn([ssPath '/Gnd'],'LConn',1));
add_block('nesl_utility/PS-Simulink Converter', [ssPath '/Vdc_PS2S'], 'Position', [180 480 220 510]);
set_param([ssPath '/Vdc_PS2S'], 'Unit', 'V');
add_line(ssPath, conn(vs,'RConn',1), conn([ssPath '/Vdc_PS2S'],'LConn',1));

% =====================================================================
%   ess_ctrl — V_dc PI + FF + slew + Rule-4 TOTAL clamp + SOC + battery
%   One discrete (50 µs) MATLAB Function, persistent states.
% =====================================================================
ctl = [ssPath '/ess_ctrl'];
add_block('simulink/User-Defined Functions/MATLAB Function', ctl, 'Position', [420 60 600 200]);
set_mlfcn(ctl, { ...
 'function [I_cmd_A, P_ess_MW, SOC, V_batt_V] = ess_ctrl(V_dc_V, Pref_MW, dt_s)'
 '%#codegen'
 '% ESS controller (IO_CONTRACTS §3.9): owns V_dc. slewed EMS feed-forward'
 '% + unslewed PI trim (H7) -> TOTAL clamp (Rule 4, SOC-aware) -> CCS cmd.'
 '% Signal-level Thevenin battery + SOC integrator (Rule 7 bounds).'
 ['V_set = ' num2str(V_dc_set_V) ';     % DC bus setpoint [V]']
 ['Kp = ' num2str(Kp_dc_MW_V) ';  Ki = ' num2str(Ki_dc_MW_Vs) ';   % V_dc PI [MW/V], [MW/V/s] (H3)']
 ['Pmax = ' num2str(P_ess_max_MW) ';  slew = ' num2str(slew_MW_s) ';   % clamp [MW], slew [MW/s]']
 ['SOCmin = ' num2str(SOC_min) ';  SOCmax = ' num2str(SOC_max) ';  E_MWh = ' num2str(E_ess_MWh) ';']
 ['eta = ' num2str(eta) ';              % each-way efficiency (H5)']
 ['Voc0 = ' num2str(V_oc_min_V) ';  dVoc = ' num2str(V_oc_max_V - V_oc_min_V) ';  Rint = ' num2str(R_int_ohm) ';']
 ['SOC0 = ' num2str(SOC_init) ';']
 '    persistent soc p_int pref_prev v_batt_prev sat_prev'
 '    if isempty(soc)'
 '        soc = SOC0; p_int = 0; pref_prev = 0;'
 '        v_batt_prev = Voc0 + SOC0*dVoc; sat_prev = 0;'
 '    end'
 ''
 '    % ── V_dc PI trim (+ = inject power into the bus = discharge)'
 '    err_V = V_set - V_dc_V;'
 '    if sat_prev < 0.5                       % Rule 7: freeze while saturated'
 '        p_int = p_int + Ki*err_V*dt_s;'
 '    end'
 '    p_int = max(-Pmax, min(Pmax, p_int));   % integral hard clamp'
 ''
 '    % ── slew limit on the EMS dispatch FF ONLY (H7): slewing the PI trim'
 '    % starves the 4.4 krad/s V_dc loop (4 kW/step) -> limit cycle.'
 '    pref_slew = max(pref_prev - slew*dt_s, min(pref_prev + slew*dt_s, Pref_MW));'
 '    pref_prev = pref_slew;'
 '    P_sum = pref_slew + Kp*err_V + p_int;   % slewed FF + unslewed trim (H4/H7)'
 ''
 '    % ── ONE total clamp (Rule 4) with SOC-aware directional limits'
 '    P_hi = Pmax;  P_lo = -Pmax;'
 '    if soc <= SOCmin, P_hi = 0; end         % empty: no more discharge'
 '    if soc >= SOCmax, P_lo = 0; end         % full : no more charge'
 '    P_act = max(P_lo, min(P_hi, P_sum));'
 '    sat_prev = double(abs(P_act - P_sum) > 1e-9);   % clamp engaged'
 ''
 '    % ── battery side (H5): discharge draws MORE than bus gets, charge'
 '    % stores LESS than bus sends. SOC integrates the CELL power.'
 '    if P_act >= 0, P_batt_MW = P_act/eta; else, P_batt_MW = P_act*eta; end'
 '    soc = soc - P_batt_MW*dt_s/(3600*E_MWh);'
 '    soc = max(SOCmin, min(SOCmax, soc));    % Rule 7 bounds'
 ''
 '    % ── Thevenin diagnostic: V_batt = OCV(SOC) − R_int·I_batt.'
 '    % Algebraic in V_batt → use previous-step V (fine at 50 µs).'
 '    V_oc = Voc0 + soc*dVoc;'
 '    I_batt = P_batt_MW*1e6 / max(v_batt_prev, 0.5*Voc0);'
 '    V_batt_V = V_oc - Rint*I_batt;'
 '    v_batt_prev = V_batt_V;'
 ''
 '    % ── bus-side current command for the CCS (+ = inject into + rail)'
 '    I_cmd_A = P_act*1e6 / max(V_dc_V, 750);'
 '    P_ess_MW = P_act;'
 '    SOC = soc;'
 'end'});
cfgC = get_param(ctl, 'MATLABFunctionConfiguration');
cfgC.UpdateMethod = 'Discrete';
cfgC.SampleTime   = num2str(dt_s);

add_block('built-in/Constant', [ssPath '/dt_c'], 'Value', num2str(dt_s), 'Position', [300 160 350 180]);
add_line(ssPath, 'Vdc_PS2S/1',   'ess_ctrl/1', 'autorouting','on');
add_line(ssPath, 'In_Pref_MW/1', 'ess_ctrl/2', 'autorouting','on');
add_line(ssPath, 'dt_c/1',       'ess_ctrl/3', 'autorouting','on');

% =====================================================================
%   CONTROLLED CURRENT SOURCE on the + rail (the whole electrical ESS)
%   Verified: RConn1=PS cmd; +cmd injects OUT of LConn1; RConn2=return.
%   Unit Delay breaks the sensor→ctrl→CCS→network algebraic loop (H6).
% =====================================================================
add_block('simulink/Discrete/Unit Delay', [ssPath '/I_delay'], ...
    'SampleTime', num2str(dt_s), 'Position', [660 95 690 125]);
add_block('nesl_utility/Simulink-PS Converter', [ssPath '/I_S2PS'], 'Position', [720 95 760 125]);
set_param([ssPath '/I_S2PS'], 'Unit', 'A', 'FilteringAndDerivatives', 'filter', ...
    'SimscapeFilterOrder', '1', 'InputFilterTimeConstant', '1e-4');
add_line(ssPath, 'ess_ctrl/1', 'I_delay/1', 'autorouting','on');
add_line(ssPath, 'I_delay/1',  'I_S2PS/1',  'autorouting','on');

ccs = [ssPath '/DCDC_CCS'];
add_block('fl_lib/Electrical/Electrical Sources/Controlled Current Source', ccs, ...
    'Position', [420 280 480 360]);
add_line(ssPath, conn([ssPath '/I_S2PS'],'RConn',1), conn(ccs,'RConn',1));  % PS cmd
add_line(ssPath, conn(ccs,'LConn',1), conn(dcPort,'RConn',1));              % + rail
add_line(ssPath, conn(ccs,'RConn',2), conn([ssPath '/Gnd'],'LConn',1));    % return (F1)

% =====================================================================
%   OUTPUTS + global Goto SOC_ESS (ESS is the §2 tag owner)
% =====================================================================
add_line(ssPath, 'ess_ctrl/2', 'Out_P_ess_MW/1', 'autorouting','on');
add_line(ssPath, 'ess_ctrl/3', 'Out_SOC_ess/1',  'autorouting','on');
add_line(ssPath, 'ess_ctrl/4', 'Out_V_batt_V/1', 'autorouting','on');
add_block('simulink/Signal Routing/Goto', [ssPath '/Goto_SOC_ESS'], ...
    'GotoTag', 'SOC_ESS', 'TagVisibility', 'global', 'Position', [1100 230 1160 248]);
add_line(ssPath, 'ess_ctrl/3', 'Goto_SOC_ESS/1', 'autorouting','on');

fprintf('  build_ess: %g MW / %g MWh BESS, V_dc PI owner (Kp=%.2g MW/V), signal-level Thevenin, SOC [%g,%g].\n', ...
    P_ess_max_MW, E_ess_MWh, Kp_dc_MW_V, SOC_min, SOC_max);
end % build_ess


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
