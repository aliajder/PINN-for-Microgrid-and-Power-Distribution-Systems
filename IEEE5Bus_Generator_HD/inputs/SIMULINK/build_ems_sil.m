%% build_ems_sil.m  — EMS embedded-C codegen + SIL (PIL prerequisite proof)
% The EMS Stateflow supervisor (build_ems.m) is the ONLY part of the microgrid
% that could run on an MCU: it is pure signal-level (no Simscape — a physical
% network can NOT be cross-compiled to a Cortex-M4). This script wraps the EMS in
% a clean Inport/Outport model, generates Embedded-Coder C, and compiles it with
% the host MinGW compiler in SIL (Software-in-the-Loop). SIL runs the SAME C that
% would be flashed to the STM32G431RB, on the PC — so it proves the controller is
% codegen-ready and bit-accurate BEFORE the on-hardware PIL step (which needs the
% STM32 support package installed; see the console note at the end).
%
% Boundary (root I/O so the whole model is a clean function):
%   In:  f_meas, SOC_ESS, SOC_H2, CB_PCC (globals, re-broadcast via Goto for the
%        EMS internal From blocks) + P_pv, P_wind, P_load
%   Out: Pref_ess, Pcmd_elec, Pcmd_fc, EMS_state
%
% Deliverables: generated C under ems_sil_ert_rtw/, a code-metrics report, and a
% Normal-vs-SIL equivalence check on one island scenario.
%
% RUN (local, wrapped by a diary so it survives an MCP drop during compile):
%   >> run('SIMULINK/build_ems_sil.m')      % or via the diary driver

clear; clc;
mdl = 'ems_sil';
ph  = @(b) get_param(b,'PortHandles');

% ── FIX: Windows hardening var NoDefaultCurrentDirectoryInExePath=1 forbids
%    cmd.exe from searching the CWD for commands. Embedded Coder's generated
%    launcher (ems_sil.bat -> call setup_mingw.bat) invokes by PLAIN name, so
%    every build dies with "'ems_sil.bat' is not recognized". Clear it for this
%    MATLAB session so the CWD is searched again (the build folder is trusted).
if ~isempty(getenv('NoDefaultCurrentDirectoryInExePath'))
    setenv('NoDefaultCurrentDirectoryInExePath','');
    fprintf('cleared NoDefaultCurrentDirectoryInExePath for the build session\n');
end

% ── CODEGEN in a CLEAN, SPACE-FREE folder ───────────────────────────────────
% The project lives in a OneDrive path full of spaces/hyphens; Embedded Coder's
% gmake launcher (ems_sil.bat) breaks on spaces ("'ems_sil.bat' is not
% recognized"). Redirect all generated code + cache to a short ASCII path.
cgRoot = 'C:\mw_codegen';
if ~isfolder(cgRoot), mkdir(cgRoot); end
% remove any stale codegen the earlier attempt left in the project tree
here0 = fileparts(mfilename('fullpath'));
for stale = {fullfile(here0,'ems_sil_ert_rtw'), fullfile(here0,'slprj')}
    if isfolder(stale{1}), rmdir(stale{1},'s'); end
end
Simulink.fileGenControl('set','CodeGenFolder',cgRoot,'CacheFolder',cgRoot, ...
    'createDir',true);
cd(cgRoot);   % build from the clean folder (cwd must not contain another codegen dir)
fprintf('codegen folder: %s\n', cgRoot);

if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);
set_param(mdl,'Solver','FixedStepDiscrete','SolverType','Fixed-step','FixedStep','0.1');
set_param(mdl,'SignalLogging','off','SaveState','off','SaveFinalState','off');

% ── the EMS organ (creates internal From globals + 3 power Inports + 4 Outports)
build_ems(mdl);

% ── root Inports for the 4 globals -> Goto(global) so EMS internal Froms resolve
glob = {'f_meas','SOC_ESS','SOC_H2','CB_PCC'};
y = 40;
for k = 1:numel(glob)
    ip = [mdl '/IN_' glob{k}];
    gt = [mdl '/GT_' glob{k}];
    add_block('simulink/Sources/In1', ip, 'Position',[20 y 50 y+14]);
    add_block('simulink/Signal Routing/Goto', gt, 'GotoTag',glob{k}, ...
        'TagVisibility','global','Position',[90 y 150 y+14]);
    add_line(mdl, ph(ip).Outport(1), ph(gt).Inport(1), 'autorouting','on');
    y = y + 30;
end

% ── root Inports for the 3 power signals -> EMS power Inports (order = 1,2,3)
pw = {'P_pv',1;'P_wind',2;'P_load',3};
for k = 1:size(pw,1)
    ip = [mdl '/IN_' pw{k,1}];
    add_block('simulink/Sources/In1', ip, 'Position',[20 y 50 y+14]);
    add_line(mdl, ph(ip).Outport(1), ph([mdl '/EMS']).Inport(pw{k,2}), 'autorouting','on');
    y = y + 30;
end

% ── root Outports for the 4 EMS outputs
outs = {'Pref_ess',1;'Pcmd_elec',2;'Pcmd_fc',3;'EMS_state',4};
y = 40;
for k = 1:size(outs,1)
    op = [mdl '/OUT_' outs{k,1}];
    add_block('simulink/Sinks/Out1', op, 'Position',[480 y 510 y+14]);
    add_line(mdl, ph([mdl '/EMS']).Outport(outs{k,2}), ph(op).Inport(1), 'autorouting','on');
    y = y + 30;
end

% ── Embedded Coder config (ERT, C, MinGW host toolchain) ────────────────────
cs = getActiveConfigSet(mdl);
set_param(cs,'SystemTargetFile','ert.tlc');
set_param(cs,'TargetLang','C');
set_param(cs,'GenerateReport',true,'GenCodeOnly',false);
set_param(cs,'ProdHWDeviceType','ARM Compatible->ARM Cortex-M');  % model the STM32 word sizes
try, set_param(cs,'PortableWordSizes',true); catch, end
set_param(cs,'Toolchain','MinGW64 | gmake (64-bit Windows)');    % host compiler for SIL
set_param(mdl,'RTWVerbose','off');

fprintf('=== EMS codegen model built. Generating embedded C (ERT + MinGW) ===\n');
rtwbuild(mdl);                          % generate + compile the C (host MinGW)
fprintf('CODEGEN_SENTINEL: OK\n');

% ── code metrics ------------------------------------------------------------
cdir = fullfile(cgRoot, [mdl '_ert_rtw']);
if isfolder(cdir)
    cf = dir(fullfile(cdir,'*.c')); hf = dir(fullfile(cdir,'*.h'));
    tot = 0; for k=1:numel(cf), tot = tot + numel(splitlines(fileread(fullfile(cdir,cf(k).name)))); end
    fprintf('\n-- generated code --\n  dir: %s\n  %d .c files, %d .h files, ~%d C lines\n', ...
        cdir, numel(cf), numel(hf), tot);
    for k=1:numel(cf), fprintf('    %s\n', cf(k).name); end
end

% ── Normal vs SIL equivalence on one island scenario ------------------------
% Scenario: islanded (CB=0), deficit ~-40 MW, SOC 0.5 -> EMS should reach STABLE
dt = 0.1; Tend = 12; t=(0:dt:Tend)';
mk = @(v) timeseries(v*ones(size(t)), t);
sig.f_meas  = timeseries([50*ones(sum(t<5),1); 49*ones(sum(t>=5 & t<8),1); 50*ones(sum(t>=8),1)], t);
sig.SOC_ESS = mk(0.50); sig.SOC_H2 = mk(0.50);
sig.CB_PCC  = timeseries([ones(sum(t<5),1); zeros(sum(t>=5),1)], t);
sig.P_pv = mk(80); sig.P_wind = mk(80); sig.P_load = mk(200);   % deficit -40

inOrder = {'f_meas','SOC_ESS','SOC_H2','CB_PCC','P_pv','P_wind','P_load'};
ds = Simulink.SimulationData.Dataset;
for k=1:numel(inOrder), ds = ds.addElement(sig.(inOrder{k}), inOrder{k}); end

set_param(mdl,'LoadExternalInput','on','ExternalInput','ds', ...
    'StopTime',num2str(Tend),'SaveOutput','on','OutputSaveName','yout', ...
    'SaveFormat','Dataset');
assignin('base','ds',ds);

fprintf('\n-- running Normal mode --\n');
set_param(mdl,'SimulationMode','normal');
oN = sim(mdl);
fprintf('-- running SIL mode (compiled C on host) --\n');
set_param(mdl,'SimulationMode','Software-in-the-loop (SIL)');
oS = sim(mdl);

% root Outports were added in this order -> yout element index order:
labels = {'Pref_ess','Pcmd_elec','Pcmd_fc','EMS_state'};
getsig = @(o,i) squeeze(o.yout{i}.Values.Data);
mism = 0;
fprintf('\n-- Normal vs SIL (max abs diff, all 4 outputs) --\n');
for i = 1:numel(labels)
    a = getsig(oN,i); b = getsig(oS,i);
    d = max(abs(a(:)-b(:)));
    fprintf('   %-10s  Normal_end=%9.3f  SIL_end=%9.3f  maxdiff=%.3e\n', labels{i}, a(end), b(end), d);
    mism = mism + (d > 1e-6);
end
set_param(mdl,'SimulationMode','normal');   % restore
fprintf('\nSIL_EQUIV (bit-accurate host C == model): %s\n', string(mism==0));
s4 = getsig(oS,4);
fprintf('EMS state trajectory (SIL): %s ... end = %d (3 = ISLAND_STABLE)\n', ...
    mat2str(unique(s4(:)).'), s4(end));
close_system(mdl,0);
fprintf('=== build_ems_sil DONE ===\n');
