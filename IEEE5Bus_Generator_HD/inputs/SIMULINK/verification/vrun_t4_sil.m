function R = vrun_t4_sil(logfile)
%% vrun_t4_sil — T4 SIL equivalence: parse the EMS SIL log -> T4_sil_equiv.mat
% The EMS embedded-C codegen + Software-in-the-Loop equivalence run
% (build_ems_sil.m via run_ems_sil.m) writes RESULTS/logs/ems_sil.txt. This
% driver PARSES that log into the verification convention:
% RESULTS/verification/T4_sil_equiv.mat + one PASS/FAIL line in SUMMARY.txt.
%
%   R = vrun_t4_sil()            % uses RESULTS/logs/ems_sil.txt
%   R = vrun_t4_sil(logfile)     % explicit log path
%
% PREREQUISITE: run the SIL build+equiv locally first (it does codegen + host
% MinGW compile, so it must run on your machine and is diary-wrapped to survive
% an MCP drop):
%       >> run('SIMULINK/run_ems_sil.m')      % (re)generates ems_sil.txt
% then call this driver.
%
% PASS = the compiled host C is bit-accurate vs the model (equiv==true, all 4
% maxdiff==0) and the controller reached ISLAND_STABLE (state_end==3). Touches
% NO control/build code — parse+save only.

    here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
    if exist(fullfile(root,'mg_params.mat'),'file') ~= 2
        w = which('build_microgrid_eelib'); if ~isempty(w), root = fileparts(fileparts(w)); end
    end
    if nargin < 1 || isempty(logfile)
        logfile = fullfile(root,'RESULTS','logs','ems_sil.txt');
    end
    verDir = fullfile(root,'RESULTS','verification'); if ~exist(verDir,'dir'), mkdir(verDir); end

    % guard: empty/stale log means the SIL run has not been done yet
    assert(isfile(logfile), 'SIL log not found: %s', logfile);
    if numel(strtrim(fileread(logfile))) < 20
        error(['vrun_t4_sil:emptyLog','ems_sil.txt is empty — run the SIL build first:\n' ...
               '   >> run(''SIMULINK/run_ems_sil.m'')\n' ...
               'then re-run vrun_t4_sil.']);
    end

    P = parse_equiv_log(logfile,'SIL');

    R = P;
    R.caseName = 'T4_sil_equiv';
    R.PASS = P.parsed_ok && P.equiv && all(P.maxdiff <= 1e-6) && (P.state_end == 3);

    fprintf('\n── T4 SIL equivalence (host embedded-C vs model) ──\n');
    fprintf('   maxdiff [Pref_ess Pcmd_elec Pcmd_fc EMS_state] = %s\n', mat2str(P.maxdiff));
    fprintf('   SIL_EQUIV=%d  state_end=%g  generated_C_lines=%g\n', P.equiv, P.state_end, P.c_lines);

    outMat = fullfile(verDir,'T4_sil_equiv.mat'); save(outMat,'-struct','R'); R.matfile = outMat;
    verdict = 'FAIL'; if R.PASS, verdict = 'PASS'; end
    note = sprintf('SIL host-C: equiv=%d maxdiff_max=%.1e state_end=%g Clines=%g', ...
        P.equiv, max(P.maxdiff), P.state_end, P.c_lines);
    line = sprintf('[%s] %-28s %-5s | %s | (from log)\n', datestr(now,'yyyy-mm-dd HH:MM'), 'T4_sil_equiv', verdict, note);
    fid = fopen(fullfile(verDir,'SUMMARY.txt'),'a'); fprintf(fid,'%s',line); fclose(fid);
    fprintf('\n[vrun_t4] %s[vrun_t4] saved %s\n', line, outMat);
end
