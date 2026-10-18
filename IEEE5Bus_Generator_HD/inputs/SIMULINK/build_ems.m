function build_ems(mdl)
%% build_ems.m  (Task 6 — the 12th and LAST module: EMS, Stateflow Layer-1)
% Builds the EMS subsystem: a RULE-BASED, 4-state Stateflow supervisor — the
% SLOW energy-management layer (~100 ms, §5.1). It does NOT do fast frequency /
% voltage control (the GFM VSC droop + A8 secondary already do that, §3.5); the
% EMS only decides "which mode + where should energy be routed". Powidian /
% Ipsakis hierarchical-supervisor logic (CLAUDE.md A6 L1, Rule 11).
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.13 (+ new §4), CLAUDE.md A6/A8/R1/R4
% Master    : SIMULINK/build_microgrid_eelib.m calls this (modules row 12).
% Params    : thresholds are LITERAL design decisions (Ipsakis-style hysteresis),
%             carried as Stateflow CONSTANT data so they are visible IN the chart.
% Pattern   : SIGNAL-LEVEL only (no Simscape / electrical) like H2Tank §3.12.
%             Reads the 4 globals via From (f_meas, SOC_ESS, SOC_H2, CB_PCC —
%             EMS owns NO tag, it only consumes), P_pv/P_wind/P_load via Inports;
%             outputs ESS/EL/FC power references + the state id.
%
%   From f_meas ─►│                         ┌─► Out_Pref_ess_MW   → ESS/In_Pref_MW
%   From SOC_ESS ►│   ┌───────────────────┐ ├─► Out_Pcmd_elec_MW  → Electrolyzer
%   From SOC_H2 ─►│   │  Stateflow chart  │ ├─► Out_Pcmd_fc_MW    → PEMFC
%   From CB_PCC ─►│──►│  4 EXCLUSIVE_OR   │─┤
%   In_P_pv_MW ──►│   │  states + hyst.   │ └─► Out_state (1..4)
%   In_P_wind_MW►│   └───────────────────┘
%   In_P_load_MW►│         dt = 0.1 s
%
% ── STATE MACHINE (A6 L1) ─────────────────────────────────────────────────
%   1 GRID_CONNECTED   (CB1=1): grid is the buffer. Surplus → battery to its
%       target band, then (economic) electrolyzer at high SOC. Deficit → import
%       from grid, FC OFF (preserve H2).
%   2 ISLAND_TRANSITION (CB1 just 0): FREEZE hydrogen (EL=0, FC held at its last
%       value — τ can't follow), Pref_ess=0 so the VSC battery droop takes over.
%       Exit: |df/dt|<rocof_settle (RoCoF settled) OR t>t_trans_max → ISLAND_STABLE.
%   3 ISLAND_STABLE    (CB1=0, settled): full coordination with Ipsakis
%       hysteresis. TIME-SCALE SPLIT — P_net is low-pass filtered (τ=tau_h2):
%       the SLOW component is served by hydrogen (EL/FC), the FAST residual by
%       the battery (the VSC droop already does the sub-second part). The FC now
%       engages on EITHER a drained battery (SOC ≤ FC_on) OR a slow deficit that
%       reaches the battery working ceiling (|pnet_lpf| ≥ FCpdon); in the latter
%       case it covers only the deficit BEYOND that ceiling, so H2 is the last
%       resort and the battery comes off its ±P_ess_max ceiling (A6/R4).
%   4 EMERGENCY        (|Δf|≥df_emerg from any state): all resources to the rail.
%       Under-frequency (deficit) → battery max discharge + FC max, EL OFF.
%       Over-frequency  (surplus) → battery max charge + EL max, FC OFF.
%       Exit: |Δf|≤df_clear → back to GRID (if reconnected) or ISLAND_STABLE.
%
%   Transitions (with hysteresis; EMERGENCY entry has highest priority):
%     GRID  →EMERG [emerg_cnt≥N] ; →TRANS [CB1<0.5]
%     TRANS →EMERG [emerg_cnt≥N] ; →GRID [CB1≥0.5] ; →STABLE [|df/dt|<0.2 ∨ t>1.5]
%     STABLE→EMERG [emerg_cnt≥N] ; →GRID [CB1≥0.5]
%     EMERG →GRID  [|Δf|≤0.8 ∧ CB1≥0.5] ; →STABLE [|Δf|≤0.8 ∧ CB1<0.5]
%   (emerg_cnt = consecutive ticks with |Δf|≥df_emerg; N=N_emerg debounce, M10.)
%   (Resync is the simple CB1 0→1 → GRID path; no PLL-resync handshake yet.)
%
% ── DESIGN DECISIONS (2026-06-27) ─────────────────────────────────────────
%  M1. STATEFLOW (not a MATLAB Function FSM). A6 L1 mandates a Stateflow chart;
%      Rule 11 forbids substituting it. Action language = MATLAB so the chart
%      body is plain MATLAB (if/end, min/max) like every other module.
%  M2. SIGNAL-LEVEL. No Simscape / Solver Configuration / Electrical Reference.
%      The EMS is pure Simulink signals (H2Tank §3.12 precedent).
%  M3. HYSTERESIS IS MANDATORY (anti-chatter). Every on/off device has DIFFERENT
%      turn-on / turn-off thresholds (deadband), latched in chart Local data
%      el_on / fc_on. A single threshold would chatter at a marginal SOC (Red
%      Flag "state machine oscillation → use hysteresis").
%  M4. THRESHOLDS as Stateflow CONSTANT data (not magic numbers buried in code)
%      → they render in the chart and are listed by the build message + the test.
%  M5. 100 ms SAMPLE TIME (§5.1 slow supervisor). Chart ChartUpdate=DISCRETE,
%      SampleTime=T_ems. dt is passed in (= T_ems) for the timer + LPF so the
%      integration is rate-exact. In the master (50 µs) AutoInsertRateTranBlk
%      bridges the boundary; the isolated test runs discrete @0.1 s (no RT).
%  M6. EMERGENCY is SIGN-AWARE. The brief lists the under-frequency action (all
%      generation max, EL off); the over-frequency dual (absorb hard) is added
%      for correctness so a surplus emergency does not command more generation.
%  M7. EMS dispatch outputs are SLOW energy setpoints. ESS gets Pref as a
%      feed-forward (its own V_dc PI does the fast trim, §3.9). The clamps here
%      are the Rule-4 TOTAL command bounds (EL 0|[5,40], FC 0|[2,30], ESS ±80).
%  M10. EMERGENCY DEBOUNCE (2026-09-10). EMERGENCY entry is gated on emerg_cnt (a
%      counter of consecutive ticks with |Δf|≥df_emerg), not on the raw instantaneous
%      |Δf|. A single 50 µs solver-step PLL spike at a load-switch discontinuity
%      (T3 verification: a 1-tick 69.7 Hz glitch at t=15 s the 100 ms EMS happened to
%      sample) otherwise trips EMERGENCY for one cycle while the physical frequency
%      stayed at 49.73 Hz (ENTSO-E-compliant, stable). The counter is incremented in
%      every non-emergency state's `during`; the guard needs N_emerg (=2 → 200 ms)
%      consecutive ticks. Genuine excursions persist and still trip (definite-time,
%      relay practice; same spirit as Rule 5 — never act on a raw measurement spike).
%  M8. SCOPE: this module emits the 3 dispatch references + state id (the task's
%      list). The VSC f_ref/V_ref/mode and the breaker command stay at their
%      §5.3 constant defaults — they belong to the DEFERRED full-model wiring /
%      island session, not to building the EMS organ. EMS reads CB1 as an input
%      (island flag); it does not initiate islanding in this scope.

% ── Thresholds (Ipsakis-style hysteresis; M4 — become chart CONSTANT data) ──
f0          = 50;     % nominal frequency [Hz]
EL_on       = 0.80;   % electrolyzer turns ON  at SOC_ESS ≥ 0.80 (+ surplus)
EL_off      = 0.75;   % electrolyzer turns OFF at SOC_ESS ≤ 0.75   (deadband 0.05)
FC_on       = 0.25;   % fuel cell    turns ON  at SOC_ESS ≤ 0.25 (+ deficit)
FC_off      = 0.30;   % fuel cell    turns OFF at SOC_ESS ≥ 0.30   (deadband 0.05)
H2_full     = 0.95;   % SOC_H2 ≥ 0.95 → block electrolyzer (tank full)
H2_empty    = 0.05;   % SOC_H2 ≤ 0.05 → block fuel cell    (tank empty)
SOC_tgt_hi  = 0.70;   % battery target band upper (grid-mode charge stops here)
P_el_min    = 5;      % electrolyzer command min nonzero [MW] (avoid stack damage)
P_el_max    = 40;     % electrolyzer command max          [MW]
P_fc_min    = 2;      % fuel cell    command min nonzero  [MW]
P_fc_max    = 30;     % fuel cell    command max          [MW]
P_ess_max   = 80;     % battery clamp ±                  [MW] (R4: < 300 MW alone)
% FC POWER-DEFICIT DISPATCH (A6/R4, added 2026-07-04). Engage the fuel cell in
% ISLAND_STABLE not ONLY when the battery is drained (SOC ≤ FC_on) but ALSO when
% the sustained (LPF) deficit approaches the battery ceiling — so a heavy island
% load is carried by H2 instead of pinning the battery at ±P_ess_max. Hysteresis
% (on-threshold > off-threshold) prevents chatter. FC then covers only the deficit
% BEYOND the working ceiling P_fc_def_on (H2 = last resort, preserve tank).
fc_pdef_on_frac  = 0.90;                       % ON  at |pnet_lpf| ≥ 0.90·P_ess_max
fc_pdef_off_frac = 0.80;                       % OFF at |pnet_lpf| ≤ 0.80·P_ess_max (deadband)
P_fc_def_on  = fc_pdef_on_frac  * P_ess_max;   % = 72 MW  (trigger + battery working ceiling FC tops up beyond)
P_fc_def_off = fc_pdef_off_frac * P_ess_max;   % = 64 MW  (turn-off deadband)
df_emerg    = 1.5;    % |Δf| ≥ → EMERGENCY                [Hz]
df_clear    = 0.8;    % |Δf| ≤ → leave EMERGENCY (hysteresis vs df_emerg) [Hz]
% EMERGENCY DEBOUNCE (M10, added 2026-09-10). |Δf|≥df_emerg must persist for this
% many consecutive EMS ticks (N·T_ems = 200 ms) before EMERGENCY trips. A single
% 50 µs solver-step PLL spike at a load-switch discontinuity (T3: a 1-tick 69.7 Hz
% glitch at t_step=15 s that the 100 ms EMS happened to sample) wrongly tripped
% EMERGENCY even though the physical nadir was 49.73 Hz (ENTSO-E-compliant) and the
% system was stable. Debouncing rejects that 1-sample artifact; a GENUINE excursion
% persists across ticks and still trips (definite-time, protection-relay practice).
% Consistent with Rule 5 (never act on a raw, unfiltered measurement transient).
N_emerg     = 2;      % consecutive |Δf|≥df_emerg ticks required to enter EMERGENCY [-]
% TRANSITION→STABLE is SETTLING-based, NOT nominal-proximity based (FIX 2026-07-05).
% Island droop leaves a STEADY f offset (e.g. ~49.07 Hz), so an |Δf|<0.5 "near
% nominal" test NEVER clears while islanded → STABLE (FC dispatch + A8 restoration)
% never runs; exit then depends solely on the timeout (which the old 5 s value put
% at the very end of a 5 s sim). Exit instead once the RoCoF has settled (|df/dt|
% small ⇒ f has reached its droop offset), with a short timeout backstop. The A8
% secondary control then pulls f→50 in STABLE. |Δf|≥df_emerg still guards EMERGENCY.
rocof_settle = 0.2;   % |df/dt| < → RoCoF settled → ISLAND_STABLE        [Hz/s]
t_trans_max  = 1.5;   % TRANSITION timeout backstop → ISLAND_STABLE       [s]
tau_h2      = 20;     % LPF time constant for the H2 time-scale split      [s]
T_ems       = 0.1;    % EMS sample time (slow supervisor, §5.1)            [s]

% ── Subsystem container ────────────────────────────────────────────────────
ssName = 'EMS';  ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 2120 360 2300]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY: From blocks for the 4 globals + 3 power Inports + dt const
% =====================================================================
add_block('simulink/Signal Routing/From', [ssPath '/From_f_meas'], ...
    'GotoTag','f_meas',  'Position',[40  60  120  78]);
add_block('simulink/Signal Routing/From', [ssPath '/From_SOC_ESS'], ...
    'GotoTag','SOC_ESS', 'Position',[40  100 120 118]);
add_block('simulink/Signal Routing/From', [ssPath '/From_SOC_H2'], ...
    'GotoTag','SOC_H2',  'Position',[40  140 120 158]);
add_block('simulink/Signal Routing/From', [ssPath '/From_CB_PCC'], ...
    'GotoTag','CB_PCC',  'Position',[40  180 120 198]);

add_block('built-in/Inport', [ssPath '/In_P_pv_MW'],   'Position',[40 220 60 238]);
add_block('built-in/Inport', [ssPath '/In_P_wind_MW'], 'Position',[40 260 60 278]);
add_block('built-in/Inport', [ssPath '/In_P_load_MW'], 'Position',[40 300 60 318]);

add_block('built-in/Constant', [ssPath '/dt_c'], 'Value', num2str(T_ems), ...
    'Position',[40 340 90 360]);

% Outputs
add_block('built-in/Outport', [ssPath '/Out_Pref_ess_MW'],  'Position',[640 90  660 108]);
add_block('built-in/Outport', [ssPath '/Out_Pcmd_elec_MW'], 'Position',[640 140 660 158]);
add_block('built-in/Outport', [ssPath '/Out_Pcmd_fc_MW'],   'Position',[640 190 660 208]);
add_block('built-in/Outport', [ssPath '/Out_state'],        'Position',[640 240 660 258]);

% =====================================================================
%   STATEFLOW CHART (M1) — empty chart, then build programmatically
% =====================================================================
chartPath = [ssPath '/EMS_Chart'];
add_block('sflib/Chart', chartPath, 'Position', [260 60 520 320]);

ch = local_get_chart(mdl, 'EMS_Chart');
ch.ActionLanguage = 'MATLAB';      % chart body is plain MATLAB (M1)
ch.ChartUpdate    = 'DISCRETE';    % M5 — slow supervisor
ch.SampleTime     = num2str(T_ems);
try, ch.UserSpecifiedStateTransitionExecutionOrder = true; catch, end

% --- Input data (ORDER defines chart input-port order; wired below) ---------
local_data(ch, 'f_meas',  'Input');
local_data(ch, 'SOC_ESS', 'Input');
local_data(ch, 'SOC_H2',  'Input');
local_data(ch, 'CB1',     'Input');
local_data(ch, 'P_pv',    'Input');
local_data(ch, 'P_wind',  'Input');
local_data(ch, 'P_load',  'Input');
local_data(ch, 'dt',      'Input');
% --- Output data (ORDER defines chart output-port order) --------------------
local_data(ch, 'Pref_ess',  'Output');
local_data(ch, 'Pcmd_elec', 'Output');
local_data(ch, 'Pcmd_fc',   'Output');
local_data(ch, 'state_id',  'Output');
% --- Local data (persistent latches / states + action temporaries) ----------
% NB: MATLAB-language charts do NOT auto-create undeclared action variables when
% built programmatically — every symbol used in an action must be declared here.
local_data(ch, 'el_on',    'Local');   % electrolyzer on/off latch (M3)
local_data(ch, 'fc_on',    'Local');   % fuel-cell    on/off latch (M3)
local_data(ch, 't_island', 'Local');   % TRANSITION timer
local_data(ch, 'pnet_lpf', 'Local');   % LPF state for the H2 time-scale split
local_data(ch, 'Pnet',     'Local');   % net power temp (gen - load)
local_data(ch, 'df',       'Local');   % frequency deviation temp (f_meas - f0)
local_data(ch, 'Prenew',   'Local');   % renewable power temp (P_pv + P_wind), GRID ladder
local_data(ch, 'Pchg',     'Local');   % battery charge temp, GRID priority ladder
local_data(ch, 'f_prev',   'Local');   % previous-cycle f_meas (RoCoF backward diff)
local_data(ch, 'rocof',    'Local');   % df/dt settling detector for TRANSITION→STABLE
local_data(ch, 'emerg_cnt','Local');   % EMERGENCY debounce: consecutive |Δf|≥df_em ticks (M10)
% --- Constant data (M4 — thresholds visible in the chart) -------------------
local_const(ch, 'f0',       f0);
local_const(ch, 'EL_on',    EL_on);
local_const(ch, 'EL_off',   EL_off);
local_const(ch, 'FC_on',    FC_on);
local_const(ch, 'FC_off',   FC_off);
local_const(ch, 'H2_full',  H2_full);
local_const(ch, 'H2_empty', H2_empty);
local_const(ch, 'Pelmin',   P_el_min);
local_const(ch, 'Pelmax',   P_el_max);
local_const(ch, 'Pfcmin',   P_fc_min);
local_const(ch, 'Pfcmax',   P_fc_max);
local_const(ch, 'Pessmax',  P_ess_max);
local_const(ch, 'FCpdon',   P_fc_def_on);    % FC power-deficit ON thr + battery working ceiling [MW]
local_const(ch, 'FCpdoff',  P_fc_def_off);   % FC power-deficit OFF thr (deadband)               [MW]
local_const(ch, 'df_em',    df_emerg);
local_const(ch, 'df_cl',    df_clear);
local_const(ch, 'rocof_se', rocof_settle);   % RoCoF settle threshold [Hz/s] (was df_se)
local_const(ch, 't_tmax',   t_trans_max);
local_const(ch, 'tau_h2',   tau_h2);
local_const(ch, 'N_emerg',  N_emerg);        % EMERGENCY debounce tick count (M10)

% =====================================================================
%   STATES (EXCLUSIVE_OR top-level)
% =====================================================================
sGrid = local_state(ch, strjoin({ ...
 'GRID_CONNECTED'
 'entry:'
 'state_id = 1;'
 'during:'
 '% GRID PRIORITY LADDER. The PV+Wind on the DC bus are a resource to USE (the'
 '% grid independently covers the AC load). Order: (1) charge battery, (2) run'
 '% electrolyzer, (3) whatever is left raises V_dc and the VSC DC-link droop'
 '% exports it to the grid (build_vsc, part A — no EMS command needed).'
 'Pnet = P_pv + P_wind - P_load;     % kept for reference only'
 'Prenew = P_pv + P_wind;            % DC generation to absorb / store / export'
 '% priority 1: charge the battery with renewables until SOC reaches EL_on'
 'Pchg = 0;'
 'if SOC_ESS < EL_on'
 'Pchg = min(Pessmax, Prenew);'
 'end'
 'Pref_ess = -Pchg;                  % - = charge'
 '% priority 2: electrolyzer absorbs the renewable BEYOND the battery, once the'
 '% battery is full enough (hysteresis) and the tank has room'
 'if SOC_ESS >= EL_on && SOC_H2 < H2_full'
 'el_on = 1;'
 'end'
 'if SOC_ESS <= EL_off || SOC_H2 >= H2_full'
 'el_on = 0;'
 'end'
 'if el_on > 0.5'
 'Pcmd_elec = min(Pelmax, max(Pelmin, Prenew - Pchg));'
 'else'
 'Pcmd_elec = 0;'
 'end'
 '% priority 3: surplus beyond battery+EL -> VSC exports (V_dc droop, part A).'
 'Pcmd_fc = 0;                       % grid mode: FC off, preserve H2'
 '% EMERGENCY debounce (M10): count consecutive ticks with |Δf|≥df_em so a single'
 '% 1-sample PLL/measurement spike at a load-switch instant cannot trip EMERGENCY;'
 '% a genuine excursion persists and still trips after N_emerg ticks (definite-time).'
 'if abs(f_meas - f0) >= df_em'
 'emerg_cnt = emerg_cnt + 1;'
 'else'
 'emerg_cnt = 0;'
 'end'
 }, newline), [40 40 220 150]);

sTrans = local_state(ch, strjoin({ ...
 'ISLAND_TRANSITION'
 'entry:'
 'state_id = 2;'
 't_island = 0;'
 'pnet_lpf = P_pv + P_wind - P_load; % seed the LPF for ISLAND_STABLE'
 'f_prev = f_meas;   % prime the RoCoF backward difference'
 'rocof  = 999;      % sentinel: block the settle-guard until a REAL df/dt exists'
 'during:'
 't_island = t_island + dt;'
 'rocof  = (f_meas - f_prev) / dt;   % RoCoF estimate [Hz/s] (settling detector)'
 'f_prev = f_meas;                   % update backward diff for next cycle'
 'Pcmd_elec = 0;     % FREEZE hydrogen (electrolyzer off)'
 'Pref_ess = 0;      % let the VSC battery droop take over; EMS does not fight'
 'el_on = 0;'
 '% Pcmd_fc is intentionally NOT written here -> the Stateflow output HOLDS its'
 '% last commanded value (FC frozen; tau_FC cannot follow the transient anyway).'
 '% EMERGENCY debounce (M10): reject a single-sample PLL spike at the islanding'
 '% instant; a genuine excursion still trips after N_emerg consecutive ticks.'
 'if abs(f_meas - f0) >= df_em'
 'emerg_cnt = emerg_cnt + 1;'
 'else'
 'emerg_cnt = 0;'
 'end'
 }, newline), [320 40 220 150]);

sStable = local_state(ch, strjoin({ ...
 'ISLAND_STABLE'
 'entry:'
 'state_id = 3;'
 'during:'
 'Pnet = P_pv + P_wind - P_load;'
 '% TIME-SCALE SPLIT: slow (LPF) part -> hydrogen, fast residual -> battery'
 'pnet_lpf = pnet_lpf + (Pnet - pnet_lpf) * dt / tau_h2;'
 '% electrolyzer hysteresis (absorb sustained surplus)'
 'if SOC_ESS >= EL_on && pnet_lpf > 0 && SOC_H2 < H2_full'
 'el_on = 1;'
 'end'
 'if SOC_ESS <= EL_off || pnet_lpf <= 0 || SOC_H2 >= H2_full'
 'el_on = 0;'
 'end'
 '% fuel-cell hysteresis (M3) — engage H2 on a SUSTAINED deficit the battery'
 '% cannot hold alone (A6/R4). Trigger on EITHER a drained battery (SOC low) OR'
 '% the slow deficit reaching the battery working ceiling FCpdon (heavy island).'
 'if (SOC_ESS <= FC_on || (-pnet_lpf) >= FCpdon) && pnet_lpf < 0 && SOC_H2 > H2_empty'
 'fc_on = 1;'
 'end'
 '% OFF only when BOTH drain reasons are clearly cleared (SOC recovered AND deficit'
 '% below the FCpdoff deadband), or there is no deficit, or the tank is empty.'
 'if (SOC_ESS >= FC_off && (-pnet_lpf) <= FCpdoff) || pnet_lpf >= 0 || SOC_H2 <= H2_empty'
 'fc_on = 0;'
 'end'
 'if el_on > 0.5'
 'Pcmd_elec = min(Pelmax, max(Pelmin, pnet_lpf));'
 'else'
 'Pcmd_elec = 0;'
 'end'
 'if fc_on > 0.5'
 'if SOC_ESS <= FC_on'
 '% drained battery -> FC carries the whole slow deficit (rest the battery)'
 'Pcmd_fc = min(Pfcmax, max(Pfcmin, -pnet_lpf));'
 'else'
 '% healthy battery carries up to its working ceiling FCpdon; FC tops up ONLY'
 '% the excess (H2 = last resort). Battery then stays off its ±Pessmax ceiling.'
 'Pcmd_fc = min(Pfcmax, max(Pfcmin, (-pnet_lpf) - FCpdon));'
 'end'
 'else'
 'Pcmd_fc = 0;'
 'end'
 '% battery holds the remaining balance (gets the fast residual)'
 'Pref_ess = max(-Pessmax, min(Pessmax, -Pnet + Pcmd_elec - Pcmd_fc));'
 '% EMERGENCY debounce (M10): a single-sample |Δf| spike (PLL glitch at a load step)'
 '% must NOT trip EMERGENCY; require N_emerg consecutive ticks (definite-time).'
 'if abs(f_meas - f0) >= df_em'
 'emerg_cnt = emerg_cnt + 1;'
 'else'
 'emerg_cnt = 0;'
 'end'
 }, newline), [320 230 220 170]);

sEmerg = local_state(ch, strjoin({ ...
 'EMERGENCY'
 'entry:'
 'state_id = 4;'
 'emerg_cnt = 0;    % M10: clear the debounce counter on entry so a STALE count'
 '                  % (frozen while in EMERGENCY, which does not update it) cannot'
 '                  % re-fire GRID/STABLE->EMERGENCY on exit -> no GRID<->EMERG chatter.'
 '                  % Re-entry then needs N_emerg FRESH consecutive over-threshold ticks.'
 'during:'
 'df = f_meas - f0;'
 'if df < 0'
 '% under-frequency (deficit): all generation to the rail, electrolyzer OFF'
 'Pref_ess = Pessmax;'
 'Pcmd_fc = Pfcmax;'
 'Pcmd_elec = 0;'
 'fc_on = 1; el_on = 0;'
 'else'
 '% over-frequency (surplus): absorb hard, FC off (M6 sign-aware dual)'
 'Pref_ess = -Pessmax;'
 'Pcmd_elec = Pelmax;'
 'Pcmd_fc = 0;'
 'el_on = 1; fc_on = 0;'
 'end'
 }, newline), [40 230 220 150]);

% =====================================================================
%   DEFAULT TRANSITION -> GRID_CONNECTED
% =====================================================================
dft = Stateflow.Transition(ch);
dft.Destination = sGrid;
dft.DestinationOClock = 0;
dft.SourceEndpoint = [sGrid.Position(1)+30, sGrid.Position(2)-40];
dft.MidPoint       = [sGrid.Position(1)+30, sGrid.Position(2)-15];

% =====================================================================
%   TRANSITIONS (ExecutionOrder: EMERGENCY entry first, then CB, then settle)
% =====================================================================
% from GRID
local_trans(ch, sGrid,  sEmerg,  '[emerg_cnt >= N_emerg]', 1, 9, 3);
local_trans(ch, sGrid,  sTrans,  '[CB1 < 0.5]',                  2, 3, 9);
% from ISLAND_TRANSITION
local_trans(ch, sTrans, sEmerg,  '[emerg_cnt >= N_emerg]', 1, 6, 12);
local_trans(ch, sTrans, sGrid,   '[CB1 >= 0.5]',                 2, 9, 3);
local_trans(ch, sTrans, sStable, '[abs(rocof) < rocof_se || t_island > t_tmax]', 3, 6, 12);
% from ISLAND_STABLE
local_trans(ch, sStable, sEmerg, '[emerg_cnt >= N_emerg]', 1, 9, 3);
local_trans(ch, sStable, sGrid,  '[CB1 >= 0.5]',                 2, 12, 6);
% from EMERGENCY
local_trans(ch, sEmerg, sGrid,   '[abs(f_meas - f0) <= df_cl && CB1 >= 0.5]', 1, 12, 6);
local_trans(ch, sEmerg, sStable, '[abs(f_meas - f0) <= df_cl && CB1 < 0.5]',  2, 3, 9);

% =====================================================================
%   ALGEBRAIC-LOOP BREAK (M9, full-model integration) — Memory on the two
%   SOC feedbacks. Closing the full model creates a direct-feedthrough loop:
%   EMS out (Pref/Pcmd) -> ESS/EL/FC ctrl -> SOC_ESS/SOC_H2 Goto -> EMS in,
%   all within one major step (the module ctrls update SOC from THIS step's
%   command). A Memory block (rate-agnostic, unlike a Unit Delay across the
%   50µs/100ms rate split) breaks it: the EMS acts on the PREVIOUS cycle's SOC
%   — physically honest for a slow supervisor. f_meas/CB1 are not in the loop.
% =====================================================================
add_block('simulink/Discrete/Memory', [ssPath '/Mem_SOC_ESS'], ...
    'InitialCondition','0.6', 'Position',[150 100 180 118]);   % IC ~ ESS SOC_init
add_block('simulink/Discrete/Memory', [ssPath '/Mem_SOC_H2'], ...
    'InitialCondition','0.5', 'Position',[150 140 180 158]);   % IC ~ tank half-full

add_line(ssPath, 'From_f_meas/1',  'EMS_Chart/1', 'autorouting','on');
add_line(ssPath, 'From_SOC_ESS/1', 'Mem_SOC_ESS/1', 'autorouting','on');
add_line(ssPath, 'Mem_SOC_ESS/1',  'EMS_Chart/2',   'autorouting','on');
add_line(ssPath, 'From_SOC_H2/1',  'Mem_SOC_H2/1',  'autorouting','on');
add_line(ssPath, 'Mem_SOC_H2/1',   'EMS_Chart/3',   'autorouting','on');
add_line(ssPath, 'From_CB_PCC/1',  'EMS_Chart/4', 'autorouting','on');
add_line(ssPath, 'In_P_pv_MW/1',   'EMS_Chart/5', 'autorouting','on');
add_line(ssPath, 'In_P_wind_MW/1', 'EMS_Chart/6', 'autorouting','on');
add_line(ssPath, 'In_P_load_MW/1', 'EMS_Chart/7', 'autorouting','on');
add_line(ssPath, 'dt_c/1',         'EMS_Chart/8', 'autorouting','on');

add_line(ssPath, 'EMS_Chart/1', 'Out_Pref_ess_MW/1',  'autorouting','on');
add_line(ssPath, 'EMS_Chart/2', 'Out_Pcmd_elec_MW/1', 'autorouting','on');
add_line(ssPath, 'EMS_Chart/3', 'Out_Pcmd_fc_MW/1',   'autorouting','on');
add_line(ssPath, 'EMS_Chart/4', 'Out_state/1',        'autorouting','on');

fprintf(['  build_ems: 4-state Stateflow EMS @%.0f ms. Hysteresis EL[on>=%.2f/off<=%.2f] ' ...
    'FC[SOC on<=%.2f/off>=%.2f | Pdef on>=%.0f/off<=%.0f MW], tank[full>=%.2f/empty<=%.2f], ' ...
    'emerg|Δf|>=%.1f/clear<=%.1f Hz. TRANS->STABLE |df/dt|<%.1f Hz/s or t>%.1f s.\n'], ...
    T_ems*1000, EL_on, EL_off, FC_on, FC_off, P_fc_def_on, P_fc_def_off, ...
    H2_full, H2_empty, df_emerg, df_clear, rocof_settle, t_trans_max);
end % build_ems


% =====================================================================
%                              HELPERS
% =====================================================================
function ch = local_get_chart(mdl, chartName)
% Robustly fetch the Stateflow.Chart object for block <mdl>/.../<chartName>.
m  = find(sfroot, '-isa', 'Stateflow.Machine', 'Name', mdl);
ch = find(m, '-isa', 'Stateflow.Chart', 'Name', chartName);
if numel(ch) ~= 1
    error('build_ems:chart', 'Expected exactly one chart named %s, found %d.', ...
        chartName, numel(ch));
end
end

function d = local_data(ch, name, scope)
d = Stateflow.Data(ch);
d.Name  = name;
d.Scope = scope;            % 'Input' | 'Output' | 'Local'
if strcmp(scope, 'Local')
    % Give every Local an explicit initial value so Stateflow treats it as
    % initialised — otherwise reading a persistent latch (el_on/fc_on) or a
    % held state (pnet_lpf) before it is written THIS step trips the strict
    % "read before write" diagnostic. The hysteresis latches NEED the prior
    % value, so this is the correct fix (not a diagnostic relaxation).
    d.Props.InitialValue = '0';
end
end

function d = local_const(ch, name, val)
d = Stateflow.Data(ch);
d.Name  = name;
d.Scope = 'Constant';
d.Props.InitialValue = num2str(val, '%.10g');
end

function s = local_state(ch, label, pos)
s = Stateflow.State(ch);
s.LabelString = label;      % first line = state name; then entry/during actions
s.Position    = pos;        % [x y w h]
end

function t = local_trans(ch, src, dst, guard, execOrder, srcClock, dstClock)
% Add a guarded transition src->dst with an explicit execution order.
t = Stateflow.Transition(ch);
t.Source            = src;
t.Destination       = dst;
t.LabelString       = guard;
t.SourceOClock      = srcClock;
t.DestinationOClock = dstClock;
try, t.ExecutionOrder = execOrder; catch, end
end
