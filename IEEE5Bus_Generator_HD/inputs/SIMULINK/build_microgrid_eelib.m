%% build_microgrid_eelib.m
% Master builder for the Hydrogen-Integrated HIL Microgrid Simulink model.
%
% Project   : YTÜ TÜBİTAK 2209-A — Real-Time HIL Simulation of Hydrogen Microgrid
% Reference : CLAUDE.md (v4), PROJECT_KNOWLEDGE.md (v2), SIMULINK/IO_CONTRACTS.md (v1.1)
% Version   : v0.1 — master skeleton + solver/reference + smoke test
%
% WHAT THIS SCRIPT DOES (v0.1)
%   1. Loads parameter MAT-files into base workspace (mg, h2, real if present).
%   2. Pushes f0 = 50 Hz into base workspace (Turkey/EU grid; library defaults
%      are 60 Hz — Rule 10 / CLAUDE.md).
%   3. Creates an empty model `microgrid_case5` with the canonical ee_lib
%      essentials in place: ONE Solver Configuration (nesl_utility), ONE
%      Electrical Reference (ee_lib).
%   4. Applies the fixed-step solver settings demanded by OPAL-RT
%      (Rule 1 / CLAUDE.md): ode14x (implicit fixed-step), FixedStep = 50 µs.
%      (Rule 1 revised 2026-06-06: ode4 diverged on the stiff ideal-source net.)
%   5. Calls module builders if they exist on the path; module-absent
%      branches are guarded so this master runs cleanly during early
%      development. See "MODULES" section for the list (per IO_CONTRACTS §3).
%   6. Saves the model to SIMULINK/microgrid_case5.slx.
%   7. Runs a minimal smoke test (open + solver params + essentials present).
%
% RUN ORDER (full pipeline)
%   run('PARAMS/case5_to_simscape.m')         % -> mg_params.mat
%   run('PARAMS/hydrogen_system_params.m')    % -> h2_params.mat
%   run('PARAMS/load_real_profiles.m')        % -> real_profiles.mat (optional v0.1)
%   run('SIMULINK/build_microgrid_eelib.m')   % -> microgrid_case5.slx
%
% DESIGN DECISIONS (locked, see CLAUDE.md A1-A6 and IO_CONTRACTS):
%   * 100% standard ee_lib + base Simulink blocks (no Dudgeon helpers).
%   * Grid-forming VSC, single common DC bus (1500 V), 33 kV AC bus.
%   * Bus 1/3/5: programmatic 2nd-order classical SM (swing eq +
%     Controlled Voltage Source). Bus 4: ideal Voltage Source (3-phase).
%   * ESS bidirectional DC-DC owns DC bus voltage regulation; VSC is
%     pure V-f droop with a saturation backstop only (IO_CONTRACTS §3.5).
%   * Goto/From restricted to 5 global signals: f_meas, SOC_ESS, SOC_H2,
%     V_pcc, CB_PCC. Everything else is direct port-to-port (CLAUDE.md).

%% ─── HOUSEKEEPING ────────────────────────────────────────────────────
clear; clc;

% Resolve project root from this script's own location so the builder
% can be called from any working directory.
scriptDir   = fileparts(mfilename('fullpath'));   % .../microh2grid/SIMULINK
projectRoot = fileparts(scriptDir);               % .../microh2grid

mdl     = 'microgrid_case5';                                        % model logical name
mdlFile = fullfile(projectRoot, 'SIMULINK', [mdl '.slx']);          % on-disk path

fprintf('\n==============================================================\n');
fprintf(' build_microgrid_eelib.m — master skeleton (v0.1)\n');
fprintf('==============================================================\n\n');

%% ─── LOAD PARAMETERS INTO BASE WORKSPACE ─────────────────────────────
% These MAT files are produced by PARAMS/case5_to_simscape.m and
% PARAMS/hydrogen_system_params.m. We push their contents into base
% workspace so block masks can reference `mg.branch.R_ohm(k)` etc.

fprintf('[1/7] Loading parameters...\n');

mgFile   = fullfile(projectRoot, 'mg_params.mat');
h2File   = fullfile(projectRoot, 'h2_params.mat');
realFile = fullfile(projectRoot, 'real_profiles.mat');

assert(exist(mgFile,'file') == 2, ...
    'mg_params.mat not found at %s. Run PARAMS/case5_to_simscape.m first.', mgFile);
assert(exist(h2File,'file') == 2, ...
    'h2_params.mat not found at %s. Run PARAMS/hydrogen_system_params.m first.', h2File);

S = load(mgFile); assignin('base', 'mg', S.mg);

% h2_params.mat stores the four hydrogen subsystem param structs at the
% top level (elec, ems, fc, tank) — not wrapped in an `h2` parent. We
% repack them into a single `h2` struct for consistent base-workspace
% access (mg.*, h2.*).
S = load(h2File);
h2 = struct();
for fn_ = fieldnames(S).'
    h2.(fn_{1}) = S.(fn_{1});
end
assignin('base', 'h2', h2);

if exist(realFile,'file') == 2
    S = load(realFile); assignin('base', 'real', S);
    fprintf('     real_profiles.mat loaded -> base workspace `real`\n');
else
    fprintf('     real_profiles.mat absent — From Workspace blocks will\n');
    fprintf('     fall back to constants until PARAMS/load_real_profiles.m runs.\n');
end

% f0 override — library defaults are 60 Hz; this project is 50 Hz.
% Any block whose nominal frequency parameter is left at default will be
% overridden via mask references to base workspace `f0`.
assignin('base', 'f0', 50);

% VSC build-time flags (read by build_vsc.m at BUILD time and baked as
% literals into the vf_droop MATLAB Function). These reproduce the GOOD
% ISLAND baseline that verify_ph3_island.m / verify_ph3_scenario.m used:
%   * vsc_inner_loop_enable=TRUE  -> FAZ3 current limiter ARMED (build_vsc.m:297).
%     Inert at normal load; engages only on overcurrent (|i_dq|>i_max_pk).
%     Default in build_vsc.m is FALSE, and this master `clear`s the base
%     workspace at start, so the flag MUST be (re)set here or the limiter
%     gets built OUT of the canonical model. (Decision 2026-07-06.)
%   * vsc_warm_start already defaults TRUE in build_vsc.m/wire_microgrid.m.
assignin('base', 'vsc_inner_loop_enable', true);
assignin('base', 'vsc_warm_start',        true);
fprintf('     base workspace: mg, h2, f0=50, vsc_inner_loop_enable=1, vsc_warm_start=1  ✓\n');

%% ─── CLEAN ANY PRIOR MODEL INSTANCE ──────────────────────────────────
fprintf('[2/7] Cleaning any prior model instance...\n');
if bdIsLoaded(mdl)
    close_system(mdl, 0);          % discard unsaved changes
    fprintf('     closed prior in-memory copy of %s\n', mdl);
end

%% ─── CREATE EMPTY MODEL ──────────────────────────────────────────────
fprintf('[3/7] Creating new model `%s`...\n', mdl);
new_system(mdl);
open_system(mdl);

%% ─── FIXED-STEP SOLVER (CLAUDE.md Rule 1, NON-NEGOTIABLE) ────────────
% ode14x (implicit fixed-step) + 50 µs. Never variable-step — OPAL-RT
% requires deterministic timing. Anything that breaks fixed-step breaks HIL.
% Rule 1 was revised 2026-06-06: ode4 (explicit RK4) DIVERGES on this
% ideal-source stiff network (Pe_LPF non-finite ~2-18 ms). ode14x is
% implicit, OPAL-RT supported, and handles the stiffness. Same FixedStep.
fprintf('[4/7] Applying fixed-step solver settings...\n');

set_param(mdl, 'SolverType',     'Fixed-step');
set_param(mdl, 'Solver',         'ode14x');
set_param(mdl, 'FixedStep',      '50e-6');
set_param(mdl, 'StartTime',      '0.0');
set_param(mdl, 'StopTime',       '10.0');               % default 10 s for smoke test
set_param(mdl, 'SimulationMode', 'normal');
set_param(mdl, 'SaveOutput',     'off');
set_param(mdl, 'SaveTime',       'off');
set_param(mdl, 'AutoInsertRateTranBlk', 'on');          % multi-rate friendliness

fprintf('     Solver=ode14x, FixedStep=50e-6 s  ✓\n');

%% ─── ee_lib ESSENTIALS (one per electrical network) ──────────────────
% Solver Configuration block: Simscape backbone, one per isolated
% electrical network. We use one here for the whole model — the AC
% transmission grid, the MG AC bus, and the DC bus are all part of the
% same connected physmod network (linked via the transformer + VSC).
%
% Electrical Reference: ground for every electrical domain. One global
% reference is enough since all electrical subsystems are connected.

fprintf('[5/7] Adding Solver Configuration + Electrical Reference...\n');

add_block('nesl_utility/Solver Configuration', ...
    [mdl '/SolverConfig'], 'Position', [40 40 80 80]);

add_block('ee_lib/Connectors & References/Electrical Reference', ...
    [mdl '/Gnd'], 'Position', [40 120 80 160]);

% Note: SolverConfig has a single Simscape port that needs to attach
% to the electrical network. We will wire it to the global ground node
% once the first electrical subsystem is built (build_transmission.m).
% Until then, both blocks sit unconnected — Simulink will warn but not
% error during model load.

%% ─── MODULE BUILDERS (guarded; absent modules are skipped) ───────────
% Each module is a function that takes the model name and adds a single
% subsystem with all internal blocks and ports per IO_CONTRACTS §3.
% Modules will be written next; this master orchestrates them so the
% same script always produces the same model.

fprintf('[6/7] Calling module builders...\n');

modules = { ...
    'build_transmission'    'TransmissionNetwork' ; ...
    'build_pcc'             'PCC'                 ; ...
    'build_mg_acbus'        'MicrogridACBus'      ; ...
    'build_vsc'             'GridFormingVSC'      ; ...
    'build_dcbus'           'DCBus'               ; ...
    'build_pv'              'PV'                  ; ...
    'build_wind'            'Wind'                ; ...
    'build_ess'             'ESS'                 ; ...
    'build_electrolyzer'    'Electrolyzer'        ; ...
    'build_fc'              'PEMFC'               ; ...
    'build_h2tank'          'H2Tank'              ; ...
    'build_ems'             'EMS'                 ...
};

built = false(size(modules,1), 1);
for k = 1:size(modules,1)
    fn = modules{k,1};
    if exist(fn, 'file') == 2
        try
            feval(fn, mdl);
            built(k) = true;
            fprintf('     %-30s -> %s  ✓\n', fn, modules{k,2});
        catch ME
            fprintf('     %-30s -> %s  ✗  (%s)\n', fn, modules{k,2}, ME.message);
        end
    else
        fprintf('     %-30s -> %s  (not yet implemented)\n', fn, modules{k,2});
    end
end

%% ─── HYDROGEN-CHAIN WIRING (master-level signal connections) ─────────
% The Electrolyzer, PEMFC and H2Tank are built as isolated subsystems
% (like every module). Close the hydrogen loop here with DIRECT port-to-
% port Simulink lines — the mdot signals are NOT among the 5 globals in
% IO_CONTRACTS §2, so they must NOT use Goto/From:
%   Electrolyzer/Out_mdot_H2_kgs (production, +) → H2Tank/In_mdot_prod_kgs
%   PEMFC/Out_mdot_H2_cons_kgs   (consumption,+) → H2Tank/In_mdot_cons_kgs
% H2Tank's SOC_H2 Goto is already global (EMS reads it via From in Task 6).
% NOTE: the DC-bus electrical inter-module wiring is a separate later step;
% this section only closes the H2 SIGNAL chain (per this session's scope).
fprintf('[6b/7] Wiring hydrogen chain (Electrolyzer/PEMFC → H2Tank)...\n');
elPath = [mdl '/Electrolyzer'];  fcPath = [mdl '/PEMFC'];  tkPath = [mdl '/H2Tank'];
haveH2chain = getSimulinkBlockHandle(elPath) > 0 ...
           && getSimulinkBlockHandle(fcPath) > 0 ...
           && getSimulinkBlockHandle(tkPath) > 0;
if haveH2chain
    phEL = get_param(elPath,'PortHandles');   % Outport(2) = Out_mdot_H2_kgs
    phFC = get_param(fcPath,'PortHandles');   % Outport(2) = Out_mdot_H2_cons_kgs
    phTK = get_param(tkPath,'PortHandles');   % Inport(1)=prod, Inport(2)=cons
    add_line(mdl, phEL.Outport(2), phTK.Inport(1), 'autorouting','on');
    add_line(mdl, phFC.Outport(2), phTK.Inport(2), 'autorouting','on');
    fprintf('     Electrolyzer/Out_mdot_H2_kgs   -> H2Tank/In_mdot_prod_kgs  ✓\n');
    fprintf('     PEMFC/Out_mdot_H2_cons_kgs     -> H2Tank/In_mdot_cons_kgs  ✓\n');
else
    fprintf('     skipped — Electrolyzer/PEMFC/H2Tank not all present.\n');
end

%% ─── FULL-MODEL WIRING (inter-module electrical + control) ───────────
% Connect the 12 modules into one running model: DC star, AC chain, the
% single Solver Configuration, EMS dispatch -> ESS/EL/FC, PV/Wind/P_load ->
% EMS, profile-source constants, VSC refs, CB cmd. See SIMULINK/wire_microgrid.m
% and IO_CONTRACTS §4. Guarded: only run when all 12 modules built (connByName
% needs every block present). D_pu stays at the module default 1.5 here; the
% grid-mode test (build_fullmodel_gridtest.m) overrides it to 60 at run time
% (test-only, Rule 12 — real damping is Task 5.2b).
fprintf('[6c/7] Full-model wiring (wire_microgrid)...\n');
if all(built) && exist('wire_microgrid','file') == 2
    try
        wire_microgrid(mdl);
    catch ME
        fprintf('     wire_microgrid FAILED: %s\n', ME.message);
    end
else
    fprintf('     skipped — not all 12 modules present (or wire_microgrid missing).\n');
end

% Auto-arrange the top-level layout for a readable canvas / screenshot (cosmetic;
% only touches the top level, never inside subsystems). Guarded — non-fatal.
fprintf('       arranging top-level layout...\n');
try
    Simulink.BlockDiagram.arrangeSystem(mdl);
catch ME
    fprintf('       arrangeSystem skipped (%s)\n', ME.message);
end

%% ─── SAVE MODEL ──────────────────────────────────────────────────────
fprintf('[7/7] Saving model to %s ...\n', mdlFile);
if ~isfolder(fullfile(projectRoot,'SIMULINK')), mkdir(fullfile(projectRoot,'SIMULINK')); end
save_system(mdl, mdlFile);
fprintf('     saved  ✓\n');

%% ─── SMOKE TEST ──────────────────────────────────────────────────────
% Minimal post-build checks. Do NOT attempt to compile/run yet — empty
% subsystems are expected in v0.1 and would fail an update_diagram call.
% We only verify the model exists in memory, the file is on disk, and
% the canonical settings are applied.

fprintf('\n==============================================================\n');
fprintf(' Smoke test\n');
fprintf('==============================================================\n');

pass = true;
check = @(cond, desc) (fprintf('  [%s] %s\n', ternary(cond,'OK ','FAIL'), desc) || true) && cond;

pass = check(bdIsLoaded(mdl),                                     'model loaded in memory') && pass;
pass = check(exist(mdlFile,'file') == 4,                          'slx file on disk') && pass;  % 4 = Simulink model
pass = check(strcmp(get_param(mdl,'Solver'),    'ode14x'),        'Solver = ode14x') && pass;
pass = check(strcmp(get_param(mdl,'SolverType'),'Fixed-step'),    'SolverType = Fixed-step') && pass;
pass = check(strcmp(get_param(mdl,'FixedStep'), '50e-6'),         'FixedStep = 50e-6') && pass;
pass = check(~isempty(find_system(mdl,'Name','SolverConfig')),    'SolverConfig block present') && pass;
pass = check(~isempty(find_system(mdl,'Name','Gnd')),             'Electrical Reference (Gnd) block present') && pass;
pass = check(evalin('base','exist(''f0'',''var'')==1 && f0==50'), 'f0 = 50 in base workspace') && pass;
pass = check(evalin('base','exist(''mg'',''var'')==1'),           'mg struct in base workspace') && pass;
pass = check(evalin('base','exist(''h2'',''var'')==1'),           'h2 struct in base workspace') && pass;

% Compile check — now that the model is fully wired it must update_diagram
% cleanly (catches solver-config / algebraic-loop / port-dimension issues).
compiled = true;
try
    set_param(mdl, 'SimulationCommand', 'update');
catch
    compiled = false;
end
pass = check(compiled, 'model COMPILES (update_diagram)') && pass;

fprintf('\nModules built: %d / %d\n', sum(built), numel(built));

fprintf('\n%s\n', repmat('=',1,62));
if pass
    fprintf(' SMOKE TEST PASSED — master skeleton ready.\n');
else
    fprintf(' SMOKE TEST FAILED — fix the above before adding modules.\n');
end
fprintf('%s\n\n', repmat('=',1,62));

%% ─── LOCAL HELPERS ───────────────────────────────────────────────────
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
