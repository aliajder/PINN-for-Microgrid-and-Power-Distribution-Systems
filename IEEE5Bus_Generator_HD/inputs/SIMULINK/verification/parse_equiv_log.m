function P = parse_equiv_log(logfile, mode)
%% parse_equiv_log — extract SIL/PIL equivalence + resource metrics from a diary
% Parses the console log produced by build_ems_sil.m / build_ems_pil.m (captured
% by run_ems_sil.m / run_ems_pil.m) into a struct — WITHOUT re-running codegen.
% Shared by vrun_t4_sil.m (SIL) and vrun_t5_pil.m (PIL).
%
%   P = parse_equiv_log(logfile, mode)
%     logfile  full path to ems_sil.txt or ems_pil.txt
%     mode     'SIL' or 'PIL' (label only; parsing is format-agnostic)
%
% Fields: mode, logfile, labels{4}, normal_end(4), tgt_end(4), maxdiff(4),
%   equiv (logical), state_end, ram_used_B/ram_KB/ram_pct, flash_used_B/
%   flash_KB/flash_pct, mcu, clock_MHz, board, elf_KB, c_lines, parsed_ok.
% On-target memory/MCU/board/elf fields are only present in a PIL log (SIL runs
% host C, no MCU) and come back NaN/'' for SIL; c_lines is SIL-only.

    if nargin < 2, mode = ''; end
    assert(isfile(logfile), 'log not found: %s', logfile);
    txt   = string(fileread(logfile));
    lines = splitlines(txt);

    P = struct('mode',upper(string(mode)),'logfile',logfile, ...
        'labels',{{'Pref_ess','Pcmd_elec','Pcmd_fc','EMS_state'}}, ...
        'normal_end',nan(1,4),'tgt_end',nan(1,4),'maxdiff',nan(1,4), ...
        'equiv',false,'state_end',NaN, ...
        'ram_used_B',NaN,'ram_KB',NaN,'ram_pct',NaN, ...
        'flash_used_B',NaN,'flash_KB',NaN,'flash_pct',NaN, ...
        'mcu','','clock_MHz',NaN,'board','','elf_KB',NaN,'c_lines',NaN,'parsed_ok',false);

    % ── per-output equivalence table (handles both SIL and PIL line formats):
    %   SIL: "Pref_ess   Normal_end=  40.000  SIL_end=  40.000  maxdiff=0.000e+00"
    %   PIL: "Pref_ess   end N=       40.000  PIL=      40.000  maxdiff=0.000e+00"
    for i = 1:numel(P.labels)
        idx = find(contains(lines,P.labels{i}) & contains(lines,'maxdiff='),1,'last');
        if isempty(idx), continue; end
        ln = lines(idx);
        md = regexp(ln,'maxdiff=([\d.eE+\-]+)','tokens','once');
        if ~isempty(md), P.maxdiff(i) = str2double(md{1}); end
        nums = regexp(ln,'=\s*([-\d.eE+]+)','tokens');   % {normal_end, tgt_end, maxdiff}
        if numel(nums) >= 2
            P.normal_end(i) = str2double(nums{1}{1});
            P.tgt_end(i)    = str2double(nums{2}{1});
        end
    end
    P.state_end = P.tgt_end(4);   % EMS_state end value (3 = ISLAND_STABLE)

    % ── overall equivalence verdict ──
    ie = find(contains(lines,'EQUIV'),1,'last');
    if ~isempty(ie)
        tf = regexp(lines(ie),':\s*(true|false)','tokens','once');
        if ~isempty(tf), P.equiv = strcmpi(tf{1},'true'); end
    end

    % ── on-target memory (PIL linker report) ──
    ram = regexp(txt,'RAM:\s*(\d+)\s*B\s*(\d+)\s*KB\s*([\d.]+)\s*%','tokens','once');
    if ~isempty(ram)
        P.ram_used_B=str2double(ram{1}); P.ram_KB=str2double(ram{2}); P.ram_pct=str2double(ram{3});
    end
    fl = regexp(txt,'FLASH:\s*(\d+)\s*B\s*(\d+)\s*KB\s*([\d.]+)\s*%','tokens','once');
    if ~isempty(fl)
        P.flash_used_B=str2double(fl{1}); P.flash_KB=str2double(fl{2}); P.flash_pct=str2double(fl{3});
    end
    mcu = regexp(txt,'MCU\s*:\s*(.+?),\s*([\d.]+)\s*MHz','tokens','once');
    if ~isempty(mcu), P.mcu=strtrim(mcu{1}); P.clock_MHz=str2double(mcu{2}); end
    bd = regexp(txt,'Board\s*:\s*(\S+)','tokens','once');
    if ~isempty(bd), P.board=strtrim(bd{1}); end
    elf = regexp(txt,'Size\s*:\s*([\d.]+)\s*KB','tokens','once');
    if ~isempty(elf), P.elf_KB=str2double(elf{1}); end
    cl = regexp(txt,'~(\d+)\s*C lines','tokens','once');   % SIL code metric
    if ~isempty(cl), P.c_lines=str2double(cl{1}); end

    P.parsed_ok = all(isfinite(P.maxdiff)) && isfinite(P.state_end);
end
