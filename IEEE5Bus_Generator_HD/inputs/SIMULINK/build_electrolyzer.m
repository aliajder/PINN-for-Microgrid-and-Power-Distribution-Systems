function build_electrolyzer(mdl)
%% build_electrolyzer.m  (Task 5.9)
% Builds the PEM Electrolyzer subsystem (Ulleberg polarization + averaged buck
% + Faraday H2 production) inside the master model. A LOAD on the DC bus.
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.10, CLAUDE.md A1/R5, PROJECT_KNOWLEDGE 2.5
% Params    : h2_params.mat struct `elec` (shared scalars, Rule 8) + Ulleberg
%             coefficients as literals (Ulleberg 2003, % TUNED for the aggregate)
% Pattern   : same signal-level CCS + Unit-Delay-breaks-algebraic-loop as
%             build_ess.m / build_pv.m — but the current is drawn FROM the bus
%             (load): I_cmd < 0 (the CCS injects negative current into the +rail).
%
% WHAT THIS MODULE IS (per §3.10):
%   One equivalent aggregate PEM stack. The EMS power command (In_Pref_MW) is
%   rate-limited + first-order-lagged (τ=30 s balance-of-plant), the buck draws
%   that power from the 1500 V bus, the Ulleberg polarization curve gives the
%   stack operating point (U_stack, I_stack), and Faraday's law converts the
%   stack current to an H2 mass-flow that feeds the H2 tank (Task 5.11).
%
%      DCBus/Elec_Elec_term ──1500V──►│ Elec_DC_in
%        In_Pref_MW ─► [clamp 5..40] ─► [slew] ─► [τ=30 s lag] ─► P_elec
%        P_elec ─► invert P=N·U_cell(I)·I (Ulleberg) ─► I_stack,U_stack
%        I_stack ─► Faraday  ṁ = η_F·N·I/(2F)·M_H2 ─► Out_mdot_H2_kgs
%        P_elec ─► I_bus = −P/V_dc  ─►(Unit delay)─► CCS  (LOAD: draws from bus)
%
% ── DESIGN DECISIONS (2026-06-20) ─────────────────────────────────────────
%  E1. ULLEBERG (2003) empirical polarization (electrolyzer_cell, a LOCAL
%      function so the P→I inversion can call it in a bisection loop):
%        U_cell = U_rev + (r1+r2·T)·i + s·log10((t1+t2/T+t3/T²)·i + 1)
%      with i = I/A [A/m²], T = 80 °C FIXED (no thermal model, R5). Terms:
%        • U_rev (1.229 V) = thermodynamic reversible threshold (ΔG/2F);
%        • (r1+r2·T)·i    = OHMIC overpotential (membrane + electrode iR drop);
%        • s·log10(...)   = ACTIVATION overpotential (Tafel/Butler-Volmer, the
%                           energy to start the reaction at the electrodes).
%      Coefficients are the classic Ulleberg set, kept for an aggregate stack;
%      the aggregate electrode area A is % TUNED so the curve lands at ~70 %
%      LHV efficiency over 5–40 MW (verified: 20 MW → 425 kg/h ≈ 21 kg/MWh).
%
%  E2. P→I INVERSION by bisection (40 iters; monotone P(I)=N·U_cell(I)·I, so the
%      root is unique). Upper bound I ≤ P_stack/(N·U_rev) since U_cell ≥ U_rev.
%      Robust, derivative-free, codegen-clean. Couples H2 to the polarization
%      (so U_stack genuinely rises with power, not a decorative curve).
%
%  E3. FARADAY (current-based, the physical law — NOT a flat kg/MWh):
%        ṁ_H2 = η_F · N_cells · I_stack / (2F) · M_H2   [kg/s]
%      2 electrons per H2 molecule. With the polarization sized to ~70 %, this
%      EMERGES as ≈21 kg/MWh (LHV basis, microgrid standard) — cross-checked in
%      the test against h2_params.elec.h2_rate_kg_per_MWh = 21.02. η_F = 0.99.
%
%  E4. τ = 30 s is BALANCE-OF-PLANT (water pump, thermal, gas–water separation),
%      NOT the sub-second electrochemistry (dropped at R5). First-order lag on
%      the realized electrical power + a finite slew (no unphysical 50 µs jump).
%
%  E5. LOAD-direction CCS (signal-level, build_ess.m H1 pattern): I_cmd =
%      −P_elec/V_dc, so the source draws power from the +rail. Unit Delay breaks
%      the V_dc→I_cmd→CCS→network→V_dc algebraic loop (ESS H6 / PV P3 lesson).
%
%  E6. Rule 4 single TOTAL clamp on the realized power: 0 (off) or [5,40] MW.
%      No PI/integrator here (pure feed-forward current-mode) → no windup path
%      (Rule 7 trivially satisfied; the lag state is bounded by the clamp).

% ── Parameters: shared scalars from h2_params (Rule 8) ─────────────────────
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
H    = load(fullfile(root,'h2_params.mat'));
el   = H.elec;  tk = H.tank;
dt_s = str2double(get_param(mdl,'FixedStep'));

N_cells   = el.n_cells;        % 400 series cells (sets stack voltage)
T_C       = el.T_C;            % 80 °C (fixed, R5)
U_rev     = el.U_rev;          % 1.229 V/cell
Fc        = el.F;              % 96485 C/mol
M_H2      = tk.M_H2;           % 0.002016 kg/mol
P_max_MW  = el.P_rated_MW;     % 40 MW
tau_s     = el.tau_s;          % 30 s (balance-of-plant)

% ── Literals: Ulleberg coefficients + aggregate area + efficiencies (% TUNED)
A_m2     = 15;        % aggregate electrode area  [m^2]  % TUNED: lands ~70% over 5-40 MW
r1       = 8.05e-5;   % ohmic   r1   [Ω·m^2]        Ulleberg (2003), Table
r2       = -2.5e-7;   % ohmic   r2   [Ω·m^2/°C]     Ulleberg (2003)
s_coef   = 0.185;     % activ.  s    [V]            Ulleberg (2003)
t1       = -0.1002;   % activ.  t1   [m^2/A]        Ulleberg (2003)
t2       = 8.424;     % activ.  t2   [m^2·°C/A]     Ulleberg (2003)
t3       = 247.3;     % activ.  t3   [m^2·°C^2/A]   Ulleberg (2003)
eta_F    = 0.99;      % Faraday efficiency          [-]   % TUNED (PEM typical)
eta_conv = 0.97;      % averaged buck efficiency     [-]   % TUNED
P_min_MW = 5;         % minimum operating power      [MW]  (below → off)
slew_MW_s= 13;        % power ramp-rate limit        [MW/s] (E4; lag dominates)
V_floor  = 750;       % min V_dc for the I=P/V_dc div [V]

% ── Subsystem container ───────────────────────────────────────────────────
ssName = 'Electrolyzer';  ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 1370 360 1510]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY PORTS
% =====================================================================
dcPort = [ssPath '/Elec_DC_in'];
add_block('built-in/PMIOPort', dcPort, 'Side', 'Left', 'Position', [40 300 60 320]);
add_block('built-in/Inport',  [ssPath '/In_Pref_MW'],     'Position', [40 90 60 108]);
add_block('built-in/Outport', [ssPath '/Out_P_elec_MW'],  'Position', [1180 80 1200 98]);
add_block('built-in/Outport', [ssPath '/Out_mdot_H2_kgs'],'Position', [1180 130 1200 148]);
add_block('built-in/Outport', [ssPath '/Out_U_stack_V'],  'Position', [1180 180 1200 198]);

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
%   el_ctrl — clamp + slew + τ=30 s lag + Ulleberg inversion + Faraday
%   One discrete (50 µs) MATLAB Function with a LOCAL electrolyzer_cell.
% =====================================================================
ctl = [ssPath '/el_ctrl'];
add_block('simulink/User-Defined Functions/MATLAB Function', ctl, 'Position', [380 60 560 200]);
set_mlfcn(ctl, { ...
 'function [I_cmd_A, P_elec_MW, mdot_H2_kgs, U_stack_V] = el_ctrl(Pref_MW, V_dc_V, dt_s)'
 '%#codegen'
 '% PEM electrolyzer (Ulleberg polarization + Faraday). Draws power from the'
 '% DC bus (LOAD: I_cmd < 0). See build_electrolyzer.m E1-E6.'
 ['N = ' num2str(N_cells) ';  A = ' num2str(A_m2) ';  T = ' num2str(T_C) ';  Urev = ' num2str(U_rev) ';']
 ['Fc = ' num2str(Fc) ';  MH2 = ' num2str(M_H2,'%.6g') ';  etaF = ' num2str(eta_F) ';  etaC = ' num2str(eta_conv) ';']
 ['Pmax = ' num2str(P_max_MW) ';  Pmin = ' num2str(P_min_MW) ';  tau = ' num2str(tau_s) ';  slew = ' num2str(slew_MW_s) ';  Vfloor = ' num2str(V_floor) ';']
 '    persistent P_act P_ramp'
 '    if isempty(P_act), P_act = 0; P_ramp = 0; end'
 ''
 '    % ── command conditioning (Rule 4 single clamp): off, or [Pmin,Pmax]'
 '    if Pref_MW < Pmin/2'
 '        P_tgt = 0;                          % off (below half-min → shut down)'
 '    else'
 '        P_tgt = min(max(Pref_MW, Pmin), Pmax);'
 '    end'
 '    % ── slew the SETPOINT toward P_tgt (independent state), then lag the'
 '    % realized power toward the slewed setpoint with the τ=30 s BoP lag (E4).'
 '    % (P_ramp must NOT be tied to P_act, or the lag never sees the target.)'
 '    dmax   = slew*dt_s;'
 '    P_ramp = max(P_ramp - dmax, min(P_ramp + dmax, P_tgt));'
 '    P_act  = P_act + (P_ramp - P_act)*dt_s/tau;'
 '    P_elec_MW = P_act;'
 ''
 '    % ── invert P_stack = N·U_cell(I)·I for the stack current (E2, bisection)'
 '    P_stack = P_act*1e6*etaC;               % stack gets bus power · η_buck'
 '    if P_stack < 1'
 '        I = 0;  U_stack_V = N*Urev;          % off: open-circuit-ish'
 '    else'
 '        lo = 0;  hi = P_stack/(N*Urev);      % U_cell ≥ Urev → I ≤ this'
 '        for it = 1:40'
 '            mid = 0.5*(lo+hi);'
 '            if N*electrolyzer_cell(mid/A, T)*mid < P_stack'
 '                lo = mid;'
 '            else'
 '                hi = mid;'
 '            end'
 '        end'
 '        I = 0.5*(lo+hi);'
 '        U_stack_V = N*electrolyzer_cell(I/A, T);'
 '    end'
 ''
 '    % ── Faraday H2 production (E3, current-based, 2 e- per H2)'
 '    mdot_H2_kgs = etaF * N * I / (2*Fc) * MH2;'
 ''
 '    % ── bus-side current command: LOAD draws from +rail → NEGATIVE (E5)'
 '    I_cmd_A = -P_act*1e6 / max(V_dc_V, Vfloor);'
 'end'
 ''
 'function U_cell = electrolyzer_cell(i, T)'
 '%#codegen'
 '% Ulleberg (2003) single-cell polarization. i = I/A [A/m^2], T [°C].'
 '% U_cell = U_rev + ohmic(i,T) + activation(i,T).'
 '    Urev = 1.229;                            % thermodynamic threshold [V]'
 '    r1 = 8.05e-5;  r2 = -2.5e-7;             % ohmic   [Ω·m^2], [Ω·m^2/°C]'
 '    s  = 0.185;                              % activation slope [V]'
 '    t1 = -0.1002;  t2 = 8.424;  t3 = 247.3;  % activation [m^2/A], ·°C, ·°C^2'
 '    r_ohm  = r1 + r2*T;'
 '    t_coef = t1 + t2/T + t3/(T*T);'
 '    arg = t_coef*i + 1;'
 '    if arg < 1, arg = 1; end                 % keep log argument ≥ 1 (i ≥ 0)'
 '    U_cell = Urev + r_ohm*i + s*log10(arg);'
 'end'});
cfgC = get_param(ctl,'MATLABFunctionConfiguration');
cfgC.UpdateMethod = 'Discrete';  cfgC.SampleTime = num2str(dt_s);

add_block('built-in/Constant', [ssPath '/dt_c'], 'Value', num2str(dt_s), 'Position', [300 170 350 190]);
add_line(ssPath, 'In_Pref_MW/1', 'el_ctrl/1', 'autorouting','on');
add_line(ssPath, 'Vdc_PS2S/1',   'el_ctrl/2', 'autorouting','on');
add_line(ssPath, 'dt_c/1',       'el_ctrl/3', 'autorouting','on');

% outputs
add_line(ssPath, 'el_ctrl/2', 'Out_P_elec_MW/1',   'autorouting','on');
add_line(ssPath, 'el_ctrl/3', 'Out_mdot_H2_kgs/1', 'autorouting','on');
add_line(ssPath, 'el_ctrl/4', 'Out_U_stack_V/1',   'autorouting','on');

% =====================================================================
%   CONTROLLED CURRENT SOURCE on the + rail (E5; load draws from bus)
%   I_cmd → Unit Delay (breaks V_dc↔I=P/V_dc↔network loop) → S-PS → CCS.
% =====================================================================
add_block('simulink/Discrete/Unit Delay', [ssPath '/I_delay'], ...
    'SampleTime', num2str(dt_s), 'Position', [660 95 690 125]);
add_block('nesl_utility/Simulink-PS Converter', [ssPath '/I_S2PS'], 'Position', [720 95 760 125]);
set_param([ssPath '/I_S2PS'], 'Unit', 'A', 'FilteringAndDerivatives', 'filter', ...
    'SimscapeFilterOrder', '1', 'InputFilterTimeConstant', '1e-4');
add_line(ssPath, 'el_ctrl/1', 'I_delay/1', 'autorouting','on');
add_line(ssPath, 'I_delay/1', 'I_S2PS/1',  'autorouting','on');

ccs = [ssPath '/EL_CCS'];
add_block('fl_lib/Electrical/Electrical Sources/Controlled Current Source', ccs, 'Position', [420 280 480 360]);
add_line(ssPath, conn([ssPath '/I_S2PS'],'RConn',1), conn(ccs,'RConn',1));   % PS cmd
add_line(ssPath, conn(ccs,'LConn',1), conn(dcPort,'RConn',1));               % + rail
add_line(ssPath, conn(ccs,'RConn',2), conn([ssPath '/Gnd'],'LConn',1));      % return (F1)

fprintf('  build_electrolyzer: PEM %g-%g MW, Ulleberg pol. (N=%d,A=%g m²,T=%g°C), τ=%g s, Faraday H2; draws from Elec_Elec_term.\n', ...
    P_min_MW, P_max_MW, N_cells, A_m2, T_C, tau_s);
end % build_electrolyzer


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
