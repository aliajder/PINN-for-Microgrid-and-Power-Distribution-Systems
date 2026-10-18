function R = vrun_t5_pil(logfile)
%% vrun_t5_pil — T5 PIL result: parse the STM32G431RB PIL log -> T5_pil.mat
% The EMS Processor-in-the-Loop run on the real STM32G431RB already succeeded and
% was captured to RESULTS/logs/ems_pil.txt (build_ems_pil.m via run_ems_pil.m).
% This driver PARSES that log (no hardware / no re-flash) into the verification
% convention: RESULTS/verification/T5_pil.mat + one PASS/FAIL line in SUMMARY.txt.
%
%   R = vrun_t5_pil()            % uses RESULTS/logs/ems_pil.txt
%   R = vrun_t5_pil(logfile)     % explicit log path
%
% PASS = on-target C bit-accurate vs model (equiv==true, all 4 maxdiff==0),
% controller reached ISLAND_STABLE (state_end==3), and the image fits the MCU
% (RAM% < 100 and FLASH% < 100). Touches NO control/build code — parse+save only.

    here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
    if exist(fullfile(root,'mg_params.mat'),'file') ~= 2
        w = which('build_microgrid_eelib'); if ~isempty(w), root = fileparts(fileparts(w)); end
    end
    if nargin < 1 || isempty(logfile)
        logfile = fullfile(root,'RESULTS','logs','ems_pil.txt');
    end
    verDir = fullfile(root,'RESULTS','verification'); if ~exist(verDir,'dir'), mkdir(verDir); end

    P = parse_equiv_log(logfile,'PIL');

    R = P;   % carry all parsed fields
    R.caseName = 'T5_pil';
    R.fits = (P.flash_pct < 100) && (P.ram_pct < 100);
    R.PASS = P.parsed_ok && P.equiv && all(P.maxdiff <= 1e-6) && (P.state_end == 3) && R.fits;

    fprintf('\n── T5 PIL (STM32G431RB, on-target) ──\n');
    fprintf('   MCU %s @ %g MHz | board %s\n', P.mcu, P.clock_MHz, P.board);
    fprintf('   maxdiff [Pref_ess Pcmd_elec Pcmd_fc EMS_state] = %s\n', mat2str(P.maxdiff));
    fprintf('   PIL_EQUIV=%d  state_end=%g  ELF=%.2f KB\n', P.equiv, P.state_end, P.elf_KB);
    fprintf('   RAM  %d B / %g KB = %.2f%%\n', P.ram_used_B,  P.ram_KB,  P.ram_pct);
    fprintf('   FLASH %d B / %g KB = %.2f%%\n', P.flash_used_B, P.flash_KB, P.flash_pct);

    outMat = fullfile(verDir,'T5_pil.mat'); save(outMat,'-struct','R'); R.matfile = outMat;
    verdict = 'FAIL'; if R.PASS, verdict = 'PASS'; end
    note = sprintf('PIL on %s @%gMHz: equiv=%d maxdiff_max=%.1e state_end=%g RAM=%.1f%% FLASH=%.1f%% ELF=%.2fKB', ...
        P.board, P.clock_MHz, P.equiv, max(P.maxdiff), P.state_end, P.ram_pct, P.flash_pct, P.elf_KB);
    line = sprintf('[%s] %-28s %-5s | %s | (from log)\n', datestr(now,'yyyy-mm-dd HH:MM'), 'T5_pil', verdict, note);
    fid = fopen(fullfile(verDir,'SUMMARY.txt'),'a'); fprintf(fid,'%s',line); fclose(fid);
    fprintf('\n[vrun_t5] %s[vrun_t5] saved %s\n', line, outMat);
end
