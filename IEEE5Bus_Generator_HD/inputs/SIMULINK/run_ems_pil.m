%% run_ems_pil.m — diary-wrapped driver for build_ems_pil.m (captures the exact
% STM32 PIL build/flash outcome or the precise missing-tool error).
% NOTE: call this with an ABSOLUTE path, e.g.
%   run('C:\...\microh2grid\SIMULINK\run_ems_pil.m')
% build_ems_pil.m cd's to C:\mw_codegen for the codegen build, so a relative
% run('SIMULINK\run_ems_pil.m') on the NEXT call fails ("Unable to change
% current folder to C:\mw_codegen\SIMULINK"). We restore the cwd below so the
% relative form keeps working after a run.
here = fileparts(mfilename('fullpath')); root = fileparts(here);
% Stash in the environment, NOT a variable: build_ems_pil.m starts with `clear`,
% and run() shares this workspace, so a plain variable would be wiped.
setenv('EMS_PIL_ORIGDIR', pwd);
addpath(fullfile(root,'PARAMS')); addpath(fullfile(root,'SIMULINK'));
logDir = fullfile(root,'RESULTS','logs'); if ~isfolder(logDir), mkdir(logDir); end
logf = fullfile(logDir,'ems_pil.txt'); if isfile(logf), delete(logf); end
diary(logf); diary on;
fprintf('=== run_ems_pil START %s ===\n', char(datetime('now')));
try
    run(fullfile(here,'build_ems_pil.m'));
    fprintf('\nPIL_DRIVER_SENTINEL: OK\n');
catch ME
    fprintf('\nPIL_DRIVER_SENTINEL: ERROR %s\n  %s\n', ME.identifier, ME.message);
    for k=1:numel(ME.stack), fprintf('   at %s (line %d)\n', ME.stack(k).name, ME.stack(k).line); end
end
cd(getenv('EMS_PIL_ORIGDIR'));   % undo build_ems_pil.m's cd to C:\mw_codegen
fprintf('=== run_ems_pil END %s (cwd restored: %s) ===\n', char(datetime('now')), pwd);
diary off;
