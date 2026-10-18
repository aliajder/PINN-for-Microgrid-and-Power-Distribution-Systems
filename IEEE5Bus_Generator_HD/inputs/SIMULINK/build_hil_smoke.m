%% build_hil_smoke.m — Task 10: real-time code-generation smoke test (Speedgoat/SLRT)
% Proves the FULL model can be built for a real-time target (Simulink Real-Time /
% Speedgoat, slrealtime.tlc). This is the concrete "FIGES can take it to HIL"
% evidence for the Speedgoat path. Diary-wrapped so it survives an MCP drop.
%
% Does NOT modify the shipped microgrid_case5.slx on disk (loads it, sets RT
% options in-memory, builds, closes without saving). Uses a space-free codegen
% folder + clears the NoDefaultCurrentDirectoryInExePath hardening var (see
% build_ems_sil.m for why).
%
% RUN:  >> run('SIMULINK/build_hil_smoke.m')   (or via a diary driver)
% LOG:  RESULTS/logs/hil_smoke.txt

here = fileparts(mfilename('fullpath')); root = fileparts(here);
addpath(fullfile(root,'PARAMS'), fullfile(root,'SIMULINK'));
logDir = fullfile(root,'RESULTS','logs'); if ~isfolder(logDir), mkdir(logDir); end
logf = fullfile(logDir,'hil_smoke.txt'); if isfile(logf), delete(logf); end
diary(logf); diary on;
fprintf('=== HIL smoke (slrealtime) START %s ===\n', char(datetime('now')));

mdl = 'microgrid_case5';
try
    % codegen env fixes (OneDrive spaces + Windows hardening var)
    if ~isempty(getenv('NoDefaultCurrentDirectoryInExePath'))
        setenv('NoDefaultCurrentDirectoryInExePath','');
    end
    cgRoot = 'C:\mw_codegen'; if ~isfolder(cgRoot), mkdir(cgRoot); end
    for stale = {fullfile(here,[mdl '_slrealtime_rtw']), fullfile(here,'slprj')}
        if isfolder(stale{1}), rmdir(stale{1},'s'); end
    end
    Simulink.fileGenControl('set','CodeGenFolder',cgRoot,'CacheFolder',cgRoot,'createDir',true);
    cd(cgRoot);

    if bdIsLoaded(mdl), close_system(mdl,0); end
    load_system(fullfile(root,'SIMULINK',[mdl '.slx']));

    % RT-safe options (in-memory only)
    set_param(mdl,'SystemTargetFile','slrealtime.tlc');
    try, set_param(mdl,'SimscapeLogType','none'); catch, end
    set_param(mdl,'SignalLogging','off','SaveOutput','off','SaveState','off','SaveFinalState','off');
    fprintf('target=%s solver=%s step=%s\n', get_param(mdl,'SystemTargetFile'), ...
        get_param(mdl,'Solver'), get_param(mdl,'FixedStep'));

    t0 = tic;
    slbuild(mdl);                     % generate + compile for the SLRT target
    fprintf('\nHIL_SMOKE_SENTINEL: OK  (%.0f s)\n', toc(t0));

    art = dir(fullfile(cgRoot,'**','*.mldatx'));
    for k=1:numel(art), fprintf('  RT app: %s (%.1f KB)\n', art(k).name, art(k).bytes/1024); end
catch ME
    fprintf('\nHIL_SMOKE_SENTINEL: ERROR %s\n  %s\n', ME.identifier, ME.message);
    for k=1:numel(ME.stack), fprintf('   at %s (line %d)\n', ME.stack(k).name, ME.stack(k).line); end
end
try, if bdIsLoaded(mdl), close_system(mdl,0); end; catch, end
fprintf('=== HIL smoke END %s ===\n', char(datetime('now')));
diary off;
