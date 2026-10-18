%% run_ems_sil.m — diary-wrapped driver for build_ems_sil.m (survives MCP drop)
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root,'PARAMS')); addpath(fullfile(root,'SIMULINK'));
logDir = fullfile(root,'RESULTS','logs'); if ~isfolder(logDir), mkdir(logDir); end
logf = fullfile(logDir,'ems_sil.txt'); if isfile(logf), delete(logf); end
diary(logf); diary on;
fprintf('=== run_ems_sil START %s ===\n', char(datetime('now')));
try
    run(fullfile(here,'build_ems_sil.m'));
    fprintf('\nDRIVER_SENTINEL: OK\n');
catch ME
    fprintf('\nDRIVER_SENTINEL: ERROR %s | %s\n', ME.identifier, ME.message);
    for k=1:numel(ME.stack), fprintf('   at %s (line %d)\n', ME.stack(k).name, ME.stack(k).line); end
end
fprintf('=== run_ems_sil END %s ===\n', char(datetime('now')));
diary off;
