%% baseline_island_210.m  — PART 0 STEP 3: clean island baseline run
% Reproduces the GOOD island baseline and records V_dc / f settling so we can
% see them oturmak (settle). LOADS the on-disk microgrid_case5.slx that STEP 2
% (rebuild_goodbase.m / build_microgrid_eelib.m) just built with ALL 4 fixes
% baked in:
%   (a) build_vsc.m  df_max_Hz = R_p*f0 = 2.5 Hz   (settle-fix, f->50 + no EMERGENCY spam)
%   (b) build_vsc.m  inner_en = 1 (FAZ3 overcurrent current-LIMITER, armed)
%   (c) build_ems.m  FC power-deficit trigger (-pnet_lpf) >= FCpdon(=72 MW)
%   (d) build_transmission.m  D_pu = 1.5 (PHYSICAL, not the test-only 60)
%
% SCENARIO (exact PART 0 spec):
%   island (CB_PCC = 0), warm-start, D_pu = 1.5,
%   P_load = 210 MW, Q_load @ pf = 0.95 lagging, StopTime = 30 s, ode14x @ 50 us.
%   SimscapeLogType = 'none', SDI off  -> avoid disk-fill on a long 50 us run.
%
% LOGGING (2026-07-02 lesson: discrete EMS-rate signals get their OWN dec=1 tap,
% never resampled through a faster tap):
%   EMS-native  (dec=1  = chart 0.1 s): Pref_ess, Pcmd_fc, EMS_state
%   electrical  (dec=20 = 1 ms):        V_dc, f_meas, P_ess_act, P_fc_act, SOC_ESS, SOC_H2
%
% OUTPUT: RESULTS/baseline_island_210.mat
% The on-disk microgrid_case5.slx is NOT modified (taps are in-memory; closed w/ 0).
%
% EXPECTED (good baseline): V_dc -> ~1500 V, f -> ~50 Hz, battery drops from ~80
% toward ~55 MW, FC ~30 MW, EMS reaches ISLAND_STABLE (state 3), NO EMERGENCY (4).
%
% RUN (local — long ~real-time-ish sim, run OUTSIDE the MCP to avoid drops):
%   >> run('SIMULINK/baseline_island_210.m')

clear; clc;
mdl  = 'microgrid_case5';

% ── robust project-root resolution (do NOT rely on mfilename alone: when this
%    script is launched via run('SIMULINK/baseline_island_210.m') from the
%    project root, mfilename('fullpath') can resolve oddly and mis-locate root).
%    Strategy: mfilename -> which(build_microgrid_eelib) -> pwd and its parent.
here = fileparts(mfilename('fullpath'));                 % expect .../microh2grid/SIMULINK
if isempty(here) || exist(fullfile(here,'build_microgrid_eelib.m'),'file') ~= 2
    w = which('build_microgrid_eelib');                 % on path since STEP 2 addpath'd SIMULINK
    if ~isempty(w), here = fileparts(w); end
end
root = fileparts(here);                                 % .../microh2grid
% final sanity: root must hold mg_params.mat; else try pwd and its parent.
if exist(fullfile(root,'mg_params.mat'),'file') ~= 2
    for cand = {pwd, fileparts(pwd)}
        if exist(fullfile(cand{1},'mg_params.mat'),'file') == 2, root = cand{1}; break; end
    end
end
slx  = fullfile(root,'SIMULINK',[mdl '.slx']);
fprintf('[baseline] resolved root = %s\n', root);
% NB: use isfile, NOT exist(...,'file')==2 — exist() returns 4 (not 2) for a
% Simulink .slx model, so the ==2 test false-negatives an existing model file.
assert(isfile(slx), ...
    ['microgrid_case5.slx not found at %s\n' ...
     'Resolved root=%s. cd to the project folder (the one holding mg_params.mat) ' ...
     'or run rebuild_goodbase.m (STEP 2) first.'], slx, root);

%% ── base workspace: params + f0 (masks reference mg/h2/f0 at sim time) ──
mgFile = fullfile(root,'mg_params.mat');
h2File = fullfile(root,'h2_params.mat');
assert(isfile(mgFile), 'mg_params.mat not found at %s', mgFile);
assert(isfile(h2File), 'h2_params.mat not found at %s', h2File);
S = load(mgFile); assignin('base','mg', S.mg);
Hs = load(h2File); h2s = struct();
fn = fieldnames(Hs); for i=1:numel(fn), h2s.(fn{i}) = Hs.(fn{i}); end
assignin('base','h2', h2s);
assignin('base','f0', 50);
% build-time flags — informational at sim time (already baked into the .slx),
% set for cleanliness / in case any mask re-reads them.
assignin('base','vsc_warm_start',        true);
assignin('base','vsc_inner_loop_enable', true);

%% ── load the freshly-built on-disk model (NOT an in-memory rebuild) ──
if bdIsLoaded(mdl), close_system(mdl, 0); end   % drop any prior in-memory copy
load_system(slx);
fprintf('[baseline] loaded on-disk %s\n', slx);

%% ── solver (Rule 1) + long-run disk-safety ──
set_param(mdl, 'SolverType',  'Fixed-step');
set_param(mdl, 'Solver',      'ode14x');
set_param(mdl, 'FixedStep',   '50e-6');
set_param(mdl, 'StartTime',   '0.0');
set_param(mdl, 'StopTime',    '30.0');
set_param(mdl, 'SimulationMode', 'normal');
set_param(mdl, 'SaveOutput',  'off');
set_param(mdl, 'SaveTime',    'off');
% disk-fill prevention on a 30 s / 50 us run (600k steps):
set_param(mdl, 'SimscapeLogType',    'none');   % no simlog (the big one)
set_param(mdl, 'SignalLogging',      'off');
set_param(mdl, 'DSMLogging',         'off');
try, set_param(mdl, 'InspectSignalLogs', 'off'); catch, end   % keep SDI out of it

%% ── SCENARIO: island, physical damping, P=210 / pf=0.95 ──
Pld_MW = 210;
Qld_MVAr = Pld_MW * tan(acos(0.95));            % = 69.02 MVAr (lagging, pf 0.95)
set_param([mdl '/Src_CB_PCC_cmd'], 'Value', '0');                 % island (CB open)
set_param([mdl '/Src_Pload_MW'],   'Value', num2str(Pld_MW));
set_param([mdl '/Src_Qload_MVAr'], 'Value', num2str(Qld_MVAr,'%.4f'));
for b = {'SM_Bus1','SM_Bus3','SM_Bus5'}
    set_param([mdl '/TransmissionNetwork/' b{1} '/D_pu'], 'Value', '1.5');   % PHYSICAL
end
fprintf('[baseline] island, D_pu=1.5, P_load=%g MW, Q_load=%.2f MVAr (pf 0.95), StopTime=30 s\n', ...
    Pld_MW, Qld_MVAr);

%% ── logging taps (own-rate: EMS dec=1, electrical dec=20) ──
y = 80;
% electrical (dec=20 = 1 ms)
tapOut (mdl, 'DCBus', 'Out_V_dc_V',  'V_dc',      20, y); y=y+40;
tapGoto(mdl, 'f_meas',               'f_meas',    20, y); y=y+40;
tapOut (mdl, 'ESS',   'Out_P_ess_MW','P_ess_act', 20, y); y=y+40;
tapOut (mdl, 'PEMFC', 'Out_P_fc_MW', 'P_fc_act',  20, y); y=y+40;
tapGoto(mdl, 'SOC_ESS',              'SOC_ESS',   20, y); y=y+40;
tapGoto(mdl, 'SOC_H2',               'SOC_H2',    20, y); y=y+40;
% EMS-native (dec=1 = chart 0.1 s rate — separate tap, NEVER resampled)
tapOut (mdl, 'EMS', 'Out_Pref_ess_MW', 'Pref_ess', 1, y); y=y+40;
tapOut (mdl, 'EMS', 'Out_Pcmd_fc_MW',  'Pcmd_fc',  1, y); y=y+40;
tapOut (mdl, 'EMS', 'Out_state',       'EMS_state',1, y); y=y+40;

%% ── compile check BEFORE the long sim ──
compiled = true;
try, set_param(mdl,'SimulationCommand','update'); catch ME
    compiled = false; fprintf('*** COMPILE FAILED: %s\n', regexprep(ME.message,'<[^>]*>','')); end
fprintf('[baseline] model COMPILES (update_diagram): %d\n', compiled);
if ~compiled, error('baseline_island_210:compileFailed','Model did not compile — aborting before sim.'); end

%% ── simulate (30 s @ 50 us) ──
fprintf('[baseline] simulating 30 s @50us ode14x, ISLAND... (this is the long one)\n');
t0 = tic; simErr = '';
try, out = sim(mdl); catch ME, simErr = ME.message; end
fprintf('[baseline] wall %.0f s\n', toc(t0));

%% ── extract + summary ──
if ~isempty(simErr)
    fprintf('*** SIM FAILED: %s\n', regexprep(simErr,'<[^>]*>',''));
    t_e=[]; Vdc=[]; f=[]; Pess=[]; Pfc=[]; socE=[]; socH=[];
    t_ems=[]; Pref=[]; PcmdFc=[]; State=[];
else
    g = @(nm) squeeze(out.get(nm).Data);
    t_e   = out.get('V_dc').Time;         % electrical timebase (1 ms)
    Vdc   = g('V_dc');
    f     = g('f_meas');
    Pess  = g('P_ess_act');
    Pfc   = g('P_fc_act');
    socE  = g('SOC_ESS');
    socH  = g('SOC_H2');
    t_ems = out.get('EMS_state').Time;    % EMS-native timebase (0.1 s)
    Pref  = g('Pref_ess');
    PcmdFc= g('Pcmd_fc');
    State = g('EMS_state');

    n5 = find(t_e >= t_e(end)-5, 1);      % last-5-s window (settled)
    fprintf('\n── settled window (last 5 s) ──\n');
    fprintf('   V_dc : end %.1f V   mean %.1f V   min/max [%.1f, %.1f]\n', ...
        Vdc(end), mean(Vdc(n5:end)), min(Vdc(n5:end)), max(Vdc(n5:end)));
    fprintf('   f    : end %.3f Hz  mean %.3f Hz  min/max [%.3f, %.3f]\n', ...
        f(end), mean(f(n5:end)), min(f(n5:end)), max(f(n5:end)));
    fprintf('   P_ess: end %.1f MW  mean %.1f MW\n', Pess(end), mean(Pess(n5:end)));
    fprintf('   P_fc : end %.1f MW  mean %.1f MW\n', Pfc(end),  mean(Pfc(n5:end)));
    fprintf('   SOC_ESS %.4f -> %.4f   SOC_H2 %.4f -> %.4f\n', socE(1),socE(end),socH(1),socH(end));
    fprintf('\n── EMS-native trace (dec=1) ──\n');
    fprintf('   EMS_state: %s\n', mat2str(State(:).'));
    fprintf('   Pref_ess : %s\n', mat2str(round(Pref(:).',1)));
    fprintf('   Pcmd_fc  : %s\n', mat2str(round(PcmdFc(:).',1)));

    fprintf('\n── PASS INDICATORS (good baseline) ──\n');
    fprintf('   V_dc settles ~1500 (|mean-1500|<75 V): %s  (%.1f V)\n', ...
        string(abs(mean(Vdc(n5:end))-1500)<75), mean(Vdc(n5:end)));
    fprintf('   f settles ~50   (|mean-50|<0.2 Hz)   : %s  (%.3f Hz)\n', ...
        string(abs(mean(f(n5:end))-50)<0.2), mean(f(n5:end)));
    fprintf('   EMS reached ISLAND_STABLE (state 3)  : %s\n', string(any(State==3)));
    fprintf('   NO EMERGENCY (state 4 never)         : %s\n', string(~any(State==4)));
    fprintf('   final EMS state                      : %d\n', State(end));
end

%% ── save + clean close (on-disk .slx untouched) ──
if ~exist(fullfile(root,'RESULTS'),'dir'), mkdir(fullfile(root,'RESULTS')); end
outMat = fullfile(root,'RESULTS','baseline_island_210.mat');
save(outMat, 't_e','Vdc','f','Pess','Pfc','socE','socH', ...
             't_ems','Pref','PcmdFc','State','simErr','Pld_MW','Qld_MVAr');
close_system(mdl, 0);   % discard in-memory taps; disk microgrid_case5.slx unchanged
fprintf('\n[baseline] saved %s\n', outMat);
fprintf('[baseline] on-disk microgrid_case5.slx NOT modified (GOODBASE intact).\n');

%% ── tap helpers (mirror verify_ph3_island.m) ──
function tapOut(mdl,ssName,portName,wsName,dec,ypos)
    ss = [mdl '/' ssName];
    k  = str2double(get_param([ss '/' portName],'Port'));
    h  = get_param(ss,'PortHandles').Outport(k);
    tw = [mdl '/TW_' wsName];
    if getSimulinkBlockHandle(tw) > 0, return; end
    add_block('simulink/Sinks/To Workspace',tw,'VariableName',wsName, ...
        'SaveFormat','Timeseries','Decimation',num2str(dec),'Position',[1100 ypos 1180 ypos+24]);
    add_line(mdl,h,get_param(tw,'PortHandles').Inport(1),'autorouting','on');
end
function tapGoto(mdl,tag,wsName,dec,ypos)
    fr = [mdl '/From_' wsName];  tw = [mdl '/TW_' wsName];
    if getSimulinkBlockHandle(tw) > 0, return; end
    add_block('simulink/Signal Routing/From', fr, 'GotoTag', tag, ...
        'Position',[1000 ypos 1060 ypos+24]);
    add_block('simulink/Sinks/To Workspace',tw,'VariableName',wsName, ...
        'SaveFormat','Timeseries','Decimation',num2str(dec),'Position',[1100 ypos 1180 ypos+24]);
    add_line(mdl, ['From_' wsName '/1'], ['TW_' wsName '/1'], 'autorouting','on');
end
