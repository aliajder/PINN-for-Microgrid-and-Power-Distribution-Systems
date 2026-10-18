function build_h2tank(mdl)
%% build_h2tank.m  (Task 5.11 — the LAST organ module, 11/12)
% Builds the H2Tank subsystem: a SIGNAL-LEVEL hydrogen storage model.
% **NO electrical content** — no Simscape, no CCS, no PS converters, no ground.
% Just a mass-balance integrator: the electrolyzer produces H2 (+), the PEMFC
% consumes it (−), the tank accumulates the difference, and reports SOC_H2 and
% the (real-gas, Z-corrected) tank pressure. The simplest module in the model.
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.12, CLAUDE.md A1/R1, PROJECT_KNOWLEDGE 2.7
% Params    : h2_params.mat struct `tank` (shared scalars, Rule 8) + Z-factor and
%             SOC bounds as literal decisions (% TUNED / % chosen).
% Pattern   : one discrete MATLAB Function with a persistent mass state (the EL/FC
%             control-function pattern), but with ZERO electrical footprint.
%
%      Electrolyzer/Out_mdot_H2_kgs ─►│ In_mdot_prod_kgs ┐
%      PEMFC/Out_mdot_H2_cons_kgs   ─►│ In_mdot_cons_kgs ┤ dm/dt = prod − cons
%                                                         │ m  ∈ [50, 950] kg
%                                                         ▼ (anti-windup freeze)
%                       Out_m_H2_kg ◄─ m
%                       Out_SOC_H2  ◄─ m/1000  ──► Goto SOC_H2 (global, §2)
%                       Out_P_tank_bar ◄─ Z·m·R·T/(M·V) /1e5
%
% ── DESIGN DECISIONS (2026-06-25) ─────────────────────────────────────────
%  T1. SIGNAL-LEVEL ONLY. The tank does not appear on any electrical network;
%      H2 mass/pressure are pure Simulink signals (the H2 chemistry is decoupled
%      from the 50 µs electrical domain — averaged-model scope, R5). So this
%      module has NO Solver Configuration / Electrical Reference / Simscape blocks.
%
%  T2. MASS BALANCE = forward-Euler discrete integrator (persistent m):
%        m(k+1) = m(k) + (mdot_prod − mdot_cons)·dt
%      Both inputs arrive as POSITIVE kg/s (EL Out_mdot_H2_kgs production, FC
%      Out_mdot_H2_cons_kgs consumption — IO_CONTRACTS §1.1 sign convention); the
%      consumption term is SUBTRACTED here. Integration is dt-exact, so the module
%      reads the model FixedStep (50 µs full model; coarse in the standalone test)
%      and is rate-agnostic — same physics at any step. (§5.1 nominally lists the
%      H2 layer at 100 ms; running at FixedStep like the EL/FC functions is
%      equivalent and avoids rate-transition blocks. Re-rate later if desired.)
%
%  T3. SOC BOUNDS [0.05, 0.95] → m ∈ [50, 950] kg (Rule 7). The clamp IS the
%      anti-windup freeze: at a rail the integrator state HOLDS, so a full tank
%      cannot keep filling and an empty tank cannot go negative — a genuine
%      physical constraint, not just a controller guard. (The EMS also manages
%      these rails at the system level — risk R1, Task 6 — but the tank protects
%      itself regardless of EMS.) IC m0 = h2_params.tank.m_init_kg = 500 kg
%      (half-full, SOC_init=0.50). SINGLE SOURCE (reconciled 2026-09): the IC is
%      read from the param, NOT a literal; h2_params.tank.SOC_init=0.50 and
%      mg_params (case5_to_simscape) both refreshed to 0.50 to agree.
%
%  T4. PRESSURE — real-gas (Z-factor) correction, NOT plain ideal gas:
%        P = Z · m · R · T / (M_H2 · V_tank)
%      H2 at 350 bar is far from ideal: the compressibility factor Z ≈ 1.2 (small
%      molecule, repulsive/co-volume regime), so the REAL pressure for a given
%      stored mass is ~20% ABOVE the ideal-gas value. Using ideal gas would
%      UNDER-state the pressure (or, read the other way, OVER-state the mass that
%      a measured 350 bar represents by ~20%). V_tank is DERIVED so that a full
%      tank (m_cap = 1000 kg) reads exactly P_max = 350 bar under the Z formula:
%        V_tank = Z·m_cap·R·T/(M_H2·P_max)  ≈ 42.2 m³
%      ⚠ This SUPERSEDES h2_params.tank.V_tank_m3 = 10 m³, which is unphysical
%      (1000 kg / 10 m³ = 100 kg/m³, vs real gaseous-H2 ≈ 23.7 kg/m³ at 350 bar /
%      25 °C, NIST). The derived 42.2 m³ → 23.7 kg/m³, physically correct. Flag to
%      update hydrogen_system_params.m (V_tank_m3 10→42.2, add Z).
%
%  T5. Out_SOC_H2 also drives Goto `SOC_H2` (global, TagVisibility=global) — the
%      H2Tank is the §2 owner of this tag (mirrors ESS owning SOC_ESS). EMS
%      (Task 6) and scopes consume it via From blocks.

% ── Parameters: shared scalars from h2_params (Rule 8) ─────────────────────
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
H    = load(fullfile(root,'h2_params.mat'));
tk   = H.tank;
dt_s = str2double(get_param(mdl,'FixedStep'));   % integrator step (T2; = model rate)

m_cap     = tk.capacity_kg;      % 1000 kg  (SOC denominator / nameplate)
P_max_bar = tk.P_max_bar;        % 350 bar
P_max_Pa  = P_max_bar*1e5;       % → Pa
TK        = tk.T_C + 273.15;     % 298.15 K (25 °C)
MH2       = tk.M_H2;             % 0.002016 kg/mol
Rgas      = tk.R_gas;            % 8.314 J/mol/K

% ── Literal decisions (Z, SOC bounds, IC) — % chosen / TUNED ───────────────
Zc      = 1.2;                   % H2 compressibility @350 bar (T4)   % chosen (NIST-consistent)
SOC_min = 0.05;  SOC_max = 0.95; % SOC limits (Rule 7, T3)
m0      = tk.m_init_kg;          % initial mass [kg] = capacity·SOC_init from h2_params
                                 % (SINGLE SOURCE = 500 kg @ SOC 0.50; no literal)
m_min   = SOC_min*m_cap;         % 50  kg
m_max   = SOC_max*m_cap;         % 950 kg
V_tank  = Zc*m_cap*Rgas*TK/(MH2*P_max_Pa);   % derived: m_cap → P_max exactly (~42.2 m³, T4)

% ── Subsystem container ───────────────────────────────────────────────────
ssName = 'H2Tank';  ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 1670 360 1810]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY PORTS (all Simulink signals — no electrical port)
% =====================================================================
add_block('built-in/Inport',  [ssPath '/In_mdot_prod_kgs'], 'Position', [40 90 60 108]);
add_block('built-in/Inport',  [ssPath '/In_mdot_cons_kgs'], 'Position', [40 150 60 168]);
add_block('built-in/Outport', [ssPath '/Out_m_H2_kg'],      'Position', [620 80 640 98]);
add_block('built-in/Outport', [ssPath '/Out_SOC_H2'],       'Position', [620 130 640 148]);
add_block('built-in/Outport', [ssPath '/Out_P_tank_bar'],   'Position', [620 180 640 198]);

% =====================================================================
%   tank_dynamics — mass balance + SOC + Z-corrected pressure.
%   One discrete (model-rate) MATLAB Function with a persistent mass state.
% =====================================================================
ctl = [ssPath '/tank_dynamics'];
add_block('simulink/User-Defined Functions/MATLAB Function', ctl, 'Position', [200 70 420 200]);
set_mlfcn(ctl, { ...
 'function [m_H2_kg, SOC_H2, P_tank_bar] = tank_dynamics(mdot_prod_kgs, mdot_cons_kgs, dt_s)'
 '%#codegen'
 '% Signal-level H2 tank (NO electrical). Mass-balance integrator + SOC + Z-'
 '% corrected real-gas pressure. See build_h2tank.m T2-T4.'
 ['m0 = ' num2str(m0) ';  m_cap = ' num2str(m_cap) ';  m_min = ' num2str(m_min) ';  m_max = ' num2str(m_max) ';']
 ['Z = ' num2str(Zc) ';  Rgas = ' num2str(Rgas) ';  TK = ' num2str(TK) ';  MH2 = ' num2str(MH2,'%.6g') ';  Vtank = ' num2str(V_tank,'%.8g') ';']
 '    persistent m'
 '    if isempty(m), m = m0; end'
 ''
 '    % ── mass balance (T2): production ADDS, consumption REMOVES.'
 '    %    Both inputs arrive as +kg/s (IO_CONTRACTS §1.1) → subtract cons.'
 '    m = m + (mdot_prod_kgs - mdot_cons_kgs)*dt_s;'
 ''
 '    % ── SOC limits [0.05,0.95] → m∈[m_min,m_max] (T3, Rule 7). The clamp is'
 '    %    the anti-windup FREEZE: at a rail the state holds, so a full tank'
 '    %    cannot overfill nor an empty tank go negative (physical constraint).'
 '    if m > m_max, m = m_max; end'
 '    if m < m_min, m = m_min; end'
 ''
 '    % ── outputs'
 '    m_H2_kg = m;'
 '    SOC_H2  = m / m_cap;                      % bounded [0.05, 0.95]'
 ''
 '    % ── real-gas pressure (T4): P = Z·m·R·T/(M·V). H2 @350 bar is NOT ideal —'
 '    %    Z≈1.2 (small molecule, repulsive regime) → real P is ~20% above the'
 '    %    ideal-gas value for the same mass. Vtank sized so m_cap → P_max.'
 '    P_tank_bar = Z * m * Rgas * TK / (MH2 * Vtank) / 1e5;'
 'end'});
cfgC = get_param(ctl,'MATLABFunctionConfiguration');
cfgC.UpdateMethod = 'Discrete';  cfgC.SampleTime = num2str(dt_s);

add_block('built-in/Constant', [ssPath '/dt_c'], 'Value', num2str(dt_s), 'Position', [120 210 170 230]);
add_line(ssPath, 'In_mdot_prod_kgs/1', 'tank_dynamics/1', 'autorouting','on');
add_line(ssPath, 'In_mdot_cons_kgs/1', 'tank_dynamics/2', 'autorouting','on');
add_line(ssPath, 'dt_c/1',             'tank_dynamics/3', 'autorouting','on');

% outputs
add_line(ssPath, 'tank_dynamics/1', 'Out_m_H2_kg/1',    'autorouting','on');
add_line(ssPath, 'tank_dynamics/2', 'Out_SOC_H2/1',     'autorouting','on');
add_line(ssPath, 'tank_dynamics/3', 'Out_P_tank_bar/1', 'autorouting','on');

% =====================================================================
%   GLOBAL Goto SOC_H2 (T5 — H2Tank is the §2 owner of this tag)
% =====================================================================
add_block('simulink/Signal Routing/Goto', [ssPath '/Goto_SOC_H2'], ...
    'GotoTag', 'SOC_H2', 'TagVisibility', 'global', 'Position', [500 220 560 238]);
add_line(ssPath, 'tank_dynamics/2', 'Goto_SOC_H2/1', 'autorouting','on');   % branch off SOC

fprintf(['  build_h2tank: signal-level tank, m0=%g kg, SOC∈[%.2f,%.2f] (m∈[%g,%g] kg), ' ...
    'Z=%.2f, V_tank=%.1f m³ (1000 kg→%g bar). Goto: SOC_H2.\n'], ...
    m0, SOC_min, SOC_max, m_min, m_max, Zc, V_tank, P_max_bar);
end % build_h2tank


% =====================================================================
%                              HELPERS
% =====================================================================
function set_mlfcn(blkPath, lines)
cfg = get_param(blkPath, 'MATLABFunctionConfiguration');
cfg.FunctionScript = strjoin(lines, newline);
end
