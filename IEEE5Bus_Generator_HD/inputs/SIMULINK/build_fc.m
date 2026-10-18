function build_fc(mdl)
%% build_fc.m  (Task 5.10)
% Builds the PEMFC subsystem (Correa polarization + averaged boost + Faraday H2
% consumption) inside the master model. A SOURCE on the DC bus — the source-
% direction twin of build_electrolyzer.m.
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.11, CLAUDE.md A1/R5, PROJECT_KNOWLEDGE 2.6
% Params    : h2_params.mat struct `fc` (shared scalars, Rule 8) + Correa
%             coefficients as literals (Correa et al. 2004, % TUNED aggregate)
%
% ── DIFFERENCES vs build_electrolyzer.m (the LOAD twin) ───────────────────
%   • DC bus SOURCE: I_cmd = +P/V_dc (injects into the +rail; EL drew −P/V_dc).
%   • BOOST converter (low stack V → 1500 V bus; EL was a buck).
%   • H2 is CONSUMED (negative tank input; EL produced).
%   • τ = 5 s (EL was 30 s) — faster BoP (air compressor + manifold fill).
%   • Polarization V_cell DROPS with current (fuel cell; EL's U_cell rose).
%   • P(I) is NON-monotonic (rises, peaks ~75 MW, falls) → the inversion
%     bisects on the RISING branch only (J ∈ [0, 1.0], well below J_peak≈1.3).
%   Same signal-level CCS + Unit-Delay loop-break + independent slew-state
%   (the EL lag-bug lesson) + LOCAL cell function.
%
%      DCBus/Elec_FC_term ◄──1500V──│ Elec_DC_out
%        In_Pref_MW ─►[clamp 2..30]─►[slew]─►[τ=5 s lag]─► P_fc
%        P_fc ─► invert P_stack=K·V_cell(J)·J (Correa, rising branch) ─► J,V_stack
%        consumption  ṁ = (P_stack/V_cell)·M_H2/(2F·η_util)  ─► Out_mdot_H2_cons_kgs
%        P_fc ─► I_bus = +P/V_dc ─►(Unit delay)─► CCS  (SOURCE: injects to bus)
%
% ── DESIGN DECISIONS (2026-06-20) ─────────────────────────────────────────
%  C1. CORREA (2004) polarization (fc_cell, LOCAL function so the P→J solve can
%      call it): V_cell = E_nernst − V_act − V_ohm − V_con, three loss regions:
%        • V_act  (activation, dominates at LOW current — Tafel ξ1..ξ4·ln(i));
%        • V_ohm  (ohmic, mid current — i·(R_M membrane + R_C contact));
%        • V_con  (concentration, HIGH current — −B·ln(1−J/J_max), gas starvation).
%      T = 75 °C fixed (R5); p_H2, p_O2 literature. The aggregate scale K is
%      % TUNED so rated 30 MW lands at V_cell≈0.70 → ~55 % LHV efficiency.
%  C2. P→J inversion by bisection on the RISING branch (J∈[0,1.0]; monotone
%      there since J_peak≈1.3). P_stack = N_s·N_p·A_cell·V_cell(J)·J = K·V_cell·J.
%  C3. Faraday CONSUMPTION (current-based, avoids the EL η-double-count):
%        ṁ = (P_stack/V_cell)·M_H2/(2F·η_util)  [kg/s]  (= total Faradaic current
%      · M_H2/2F / utilization). With V_cell≈0.70 this is ≈54.6 kg/MWh (LHV).
%  C4. τ = 5 s BoP (air compressor + manifold/O2 settling), not electrochemistry
%      (R5). Independent slewed setpoint state THEN first-order lag (EL bug fix).
%  C5. SOURCE-direction CCS: I_cmd = +P/V_dc injects into the +rail. Unit Delay
%      breaks the V_dc→I→CCS→network loop (ESS H6 / PV / EL lesson).
%  C6. Rule 4 single clamp (0 or [2,30] MW); no PI → no windup (Rule 7 trivial).

% ── Parameters: shared scalars from h2_params (Rule 8) ─────────────────────
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
H    = load(fullfile(root,'h2_params.mat'));
fc   = H.fc;  tk = H.tank;
dt_s = str2double(get_param(mdl,'FixedStep'));

N_s       = fc.n_cells;        % 600 series cells
A_cm2     = fc.A_cm2;          % 500 cm^2 per cell
T_C       = fc.T_C;            % 75 °C (fixed, R5)
Fc        = 96485;             % C/mol
M_H2      = tk.M_H2;           % 0.002016 kg/mol
P_max_MW  = fc.P_rated_MW;     % 30 MW
tau_s     = fc.tau_s;          % 5 s
J_max     = fc.i_L_Acm2;       % 1.5 A/cm^2 (limiting current density, V_con)

% ── Literals: aggregate scale + Correa coeffs + efficiencies (% TUNED) ─────
N_p      = 408;       % parallel strings (aggregate)   [-]   % TUNED: 30 MW → V_cell≈0.70
K_agg    = N_s*N_p*A_cm2;   % P_stack = K·V_cell(J)·J
xi1      = -0.948;    % Correa activation ξ1   [V]      Correa et al. (2004)
xi3      = 7.6e-5;    % Correa activation ξ3            Correa (2004)
xi4      = -1.93e-4;  % Correa activation ξ4            Correa (2004)
B_con    = 0.016;     % concentration coeff B  [V]      Correa (2004)
R_M      = 0.00019;   % membrane resistance    [Ω]      (R_M+R_C = h2_params R_int=2.4e-4)
R_C      = 0.00005;   % contact resistance     [Ω]
p_H2     = 1.5;       % H2 partial pressure    [atm]    % literature
p_O2     = 1.0;       % O2 partial pressure    [atm]    % literature
eta_util = 0.98;      % H2 fuel utilization    [-]      % TUNED (→ ~54.6 kg/MWh)
eta_conv = 0.97;      % averaged boost eff.    [-]      % TUNED
P_min_MW = 2;         % minimum operating power [MW]
slew_MW_s= 12;        % power ramp-rate limit  [MW/s]   (C4; lag dominates)
J_hi     = 1.0;       % bisection cap (rising branch, < J_peak≈1.3) [A/cm^2]
V_floor  = 750;       % min V_dc for I=P/V_dc div [V]

% ── Subsystem container ───────────────────────────────────────────────────
ssName = 'PEMFC';  ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 1520 360 1660]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY PORTS
% =====================================================================
dcPort = [ssPath '/Elec_DC_out'];
add_block('built-in/PMIOPort', dcPort, 'Side', 'Left', 'Position', [40 300 60 320]);
add_block('built-in/Inport',  [ssPath '/In_Pref_MW'],          'Position', [40 90 60 108]);
add_block('built-in/Outport', [ssPath '/Out_P_fc_MW'],         'Position', [1180 80 1200 98]);
add_block('built-in/Outport', [ssPath '/Out_mdot_H2_cons_kgs'],'Position', [1180 130 1200 148]);
add_block('built-in/Outport', [ssPath '/Out_V_stack_V'],       'Position', [1180 180 1200 198]);

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
%   fc_ctrl — clamp + slew + τ=5 s lag + Correa inversion + Faraday consumption
% =====================================================================
ctl = [ssPath '/fc_ctrl'];
add_block('simulink/User-Defined Functions/MATLAB Function', ctl, 'Position', [380 60 560 200]);
set_mlfcn(ctl, { ...
 'function [I_cmd_A, P_fc_MW, mdot_cons_kgs, V_stack_V] = fc_ctrl(Pref_MW, V_dc_V, dt_s)'
 '%#codegen'
 '% PEMFC (Correa polarization + Faraday consumption). SOURCES power to the DC'
 '% bus (I_cmd > 0). See build_fc.m C1-C6.'
 ['Ns = ' num2str(N_s) ';  K = ' num2str(K_agg,'%.8g') ';  Fc = ' num2str(Fc) ';  MH2 = ' num2str(M_H2,'%.6g') ';']
 ['etaU = ' num2str(eta_util) ';  etaC = ' num2str(eta_conv) ';  Jhi = ' num2str(J_hi) ';']
 ['Pmax = ' num2str(P_max_MW) ';  Pmin = ' num2str(P_min_MW) ';  tau = ' num2str(tau_s) ';  slew = ' num2str(slew_MW_s) ';  Vfloor = ' num2str(V_floor) ';']
 '    persistent P_act P_ramp'
 '    if isempty(P_act), P_act = 0; P_ramp = 0; end'
 ''
 '    % ── command conditioning (Rule 4 single clamp): off, or [Pmin,Pmax]'
 '    if Pref_MW < Pmin/2'
 '        P_tgt = 0;'
 '    else'
 '        P_tgt = min(max(Pref_MW, Pmin), Pmax);'
 '    end'
 '    % ── independent slewed setpoint, then τ=5 s BoP lag (C4; EL bug lesson)'
 '    dmax   = slew*dt_s;'
 '    P_ramp = max(P_ramp - dmax, min(P_ramp + dmax, P_tgt));'
 '    P_act  = P_act + (P_ramp - P_act)*dt_s/tau;'
 '    P_fc_MW = P_act;'
 ''
 '    % ── invert P_stack = K·V_cell(J)·J for J on the RISING branch (C2)'
 '    P_stack = P_act*1e6/etaC;                % stack must make bus power / η_boost'
 '    if P_stack < 1'
 '        Jsol = 0;  mdot_cons_kgs = 0;'
 '    else'
 '        lo = 0;  hi = Jhi;'
 '        for it = 1:40'
 '            mid = 0.5*(lo+hi);'
 '            if K*fc_cell(mid)*mid < P_stack'
 '                lo = mid;'
 '            else'
 '                hi = mid;'
 '            end'
 '        end'
 '        Jsol = 0.5*(lo+hi);'
 '        Vc = fc_cell(Jsol);'
 '        mdot_cons_kgs = (P_stack/Vc) * MH2 / (2*Fc*etaU);   % Faraday consumption (C3)'
 '    end'
 '    V_stack_V = Ns * fc_cell(max(Jsol, 1e-3));    % stack voltage (OCV-ish when off)'
 ''
 '    % ── bus-side current command: SOURCE injects into +rail → POSITIVE (C5)'
 '    I_cmd_A = P_act*1e6 / max(V_dc_V, Vfloor);'
 'end'
 ''
 'function V_cell = fc_cell(J)'
 '%#codegen'
 '% Correa et al. (2004) PEMFC single-cell polarization. J = current density'
 '% [A/cm^2]. V_cell = E_nernst − V_act − V_ohm − V_con (DROPS with current).'
 '    T = 75;  TK = T + 273.15;  A = 500;  pH2 = 1.5;  pO2 = 1.0;  Jmax = 1.5;'
 '    x1 = -0.948;  x3 = 7.6e-5;  x4 = -1.93e-4;  B = 0.016;  RM = 0.00019;  RC = 0.00005;'
 '    E = 1.229 - 0.85e-3*(TK-298.15) + 4.3085e-5*TK*(log(pH2) + 0.5*log(pO2));'
 '    C_O2 = pO2/(5.08e6*exp(-498/TK));'
 '    C_H2 = pH2/(1.09e6*exp(77/TK));'
 '    x2 = 0.00286 + 0.0002*log(A) + 4.3e-5*log(C_H2);'
 '    iFC = max(J*A, 1e-6);                          % per-cell current [A]'
 '    Vact = -(x1 + x2*TK + x3*TK*log(C_O2) + x4*TK*log(iFC));   % activation loss'
 '    Vohm = iFC*(RM + RC);                          % ohmic loss'
 '    Vcon = -B*log(max(1 - J/Jmax, 1e-6));          % concentration loss'
 '    V_cell = E - Vact - Vohm - Vcon;'
 'end'});
cfgC = get_param(ctl,'MATLABFunctionConfiguration');
cfgC.UpdateMethod = 'Discrete';  cfgC.SampleTime = num2str(dt_s);

add_block('built-in/Constant', [ssPath '/dt_c'], 'Value', num2str(dt_s), 'Position', [300 170 350 190]);
add_line(ssPath, 'In_Pref_MW/1', 'fc_ctrl/1', 'autorouting','on');
add_line(ssPath, 'Vdc_PS2S/1',   'fc_ctrl/2', 'autorouting','on');
add_line(ssPath, 'dt_c/1',       'fc_ctrl/3', 'autorouting','on');

% outputs
add_line(ssPath, 'fc_ctrl/2', 'Out_P_fc_MW/1',          'autorouting','on');
add_line(ssPath, 'fc_ctrl/3', 'Out_mdot_H2_cons_kgs/1', 'autorouting','on');
add_line(ssPath, 'fc_ctrl/4', 'Out_V_stack_V/1',        'autorouting','on');

% =====================================================================
%   CONTROLLED CURRENT SOURCE on the + rail (C5; source injects to bus)
% =====================================================================
add_block('simulink/Discrete/Unit Delay', [ssPath '/I_delay'], ...
    'SampleTime', num2str(dt_s), 'Position', [660 95 690 125]);
add_block('nesl_utility/Simulink-PS Converter', [ssPath '/I_S2PS'], 'Position', [720 95 760 125]);
set_param([ssPath '/I_S2PS'], 'Unit', 'A', 'FilteringAndDerivatives', 'filter', ...
    'SimscapeFilterOrder', '1', 'InputFilterTimeConstant', '1e-4');
add_line(ssPath, 'fc_ctrl/1', 'I_delay/1', 'autorouting','on');
add_line(ssPath, 'I_delay/1', 'I_S2PS/1',  'autorouting','on');

ccs = [ssPath '/FC_CCS'];
add_block('fl_lib/Electrical/Electrical Sources/Controlled Current Source', ccs, 'Position', [420 280 480 360]);
add_line(ssPath, conn([ssPath '/I_S2PS'],'RConn',1), conn(ccs,'RConn',1));   % PS cmd
add_line(ssPath, conn(ccs,'LConn',1), conn(dcPort,'RConn',1));               % + rail
add_line(ssPath, conn(ccs,'RConn',2), conn([ssPath '/Gnd'],'LConn',1));      % return (F1)

fprintf('  build_fc: PEMFC %g-%g MW, Correa pol. (Ns=%d,Np=%d,T=%g°C), τ=%g s, Faraday H2 consume; sources to Elec_FC_term.\n', ...
    P_min_MW, P_max_MW, N_s, N_p, T_C, tau_s);
end % build_fc


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
