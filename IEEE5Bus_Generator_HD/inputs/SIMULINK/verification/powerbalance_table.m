function T = powerbalance_table(varargin)
%% powerbalance_table — assemble the T1/T2/T3 island power-balance table
% Builds the publication table requested (rows {Load, PV, Wind, Battery, FC,
% Electrolyzer, VSC, Loss}; cols {T1, T2, T3}; MW with signs) from the MEASURED
% last-5 s means recorded by vrun_island.m / vrun_island_step.m, prints it, runs
% the power-balance check per column, and writes RESULTS/verification/
% POWER_BALANCE.txt.
%
% USAGE
%   T = powerbalance_table(R1, R2, R3)        % from returned R structs
%   T = powerbalance_table()                  % load the 3 saved .mat files
%   T = powerbalance_table('T1.mat','T2.mat','T3.mat')  % explicit paths
%
% Sign convention (publication): generation & storage-discharge (+),
% load & consumption (-). Loss is a SINK, printed with a - sign.
%
% This is a REPORT-ONLY helper: it reads recorded fields, touches no model.

    %% ── resolve the three result structs ──
    if nargin == 3 && isstruct(varargin{1})
        R = {varargin{1}, varargin{2}, varargin{3}};
    else
        verDir = local_verdir();
        if nargin == 3
            files = varargin;
        else
            files = {fullfile(verDir,'T1_island_baseline.mat'), ...
                     fullfile(verDir,'T2_island_light.mat'), ...
                     fullfile(verDir,'T3_h2_dispatch.mat')};
        end
        R = cell(1,3);
        for k = 1:3
            assert(isfile(files{k}), 'result file not found: %s', files{k});
            R{k} = load(files{k});
        end
    end
    lbl = {'T1','T2','T3'};

    %% ── pull the measured last-5 s means (NaN if a run failed) ──
    n = 3;
    Load=nan(1,n); PV=nan(1,n); Wind=nan(1,n); Batt=nan(1,n);
    FC=nan(1,n); Elec=nan(1,n); VSC=nan(1,n); Loss=nan(1,n);
    Lossdc=nan(1,n); Lossac=nan(1,n); Vdc=nan(1,n); f=nan(1,n);
    for k = 1:n
        Load(k) = gv(R{k},'Pload_MW');
        PV(k)   = gv(R{k},'Ppv_mean');
        Wind(k) = gv(R{k},'Pwind_mean');
        Batt(k) = gv(R{k},'Pess_mean');
        FC(k)   = gv(R{k},'Pfc_mean');
        Elec(k) = gv(R{k},'Pelec_mean');
        VSC(k)  = gv(R{k},'Pvsc_mean');
        Loss(k) = gv(R{k},'Ploss_MW');
        Lossdc(k)=gv(R{k},'Lossdc_MW');
        Lossac(k)=gv(R{k},'Lossac_MW');
        Vdc(k)  = gv(R{k},'Vdc_mean');
        f(k)    = gv(R{k},'f_mean');
    end

    %% ── print the signed table ──
    fprintf('\n================ ISLAND POWER BALANCE (last 5 s mean, MW) ================\n');
    fprintf('  sign convention: generation & storage-discharge (+), load & consumption (-)\n\n');
    fprintf('  %-14s %10s %10s %10s\n', 'Row', lbl{1}, lbl{2}, lbl{3});
    fprintf('  %s\n', repmat('-',1,48));
    prow('Load (-)',        -Load);
    prow('PV (+)',           PV);
    prow('Wind (+)',         Wind);
    prow('Battery (+)',      Batt);
    prow('FC (+)',           FC);
    prow('Electrolyzer (-)', -Elec);
    prow('VSC (+)',          VSC);
    prow('Loss (-)',        -Loss);

    %% ── per-column power-balance check ──
    fprintf('\n  %s\n', repmat('-',1,48));
    resid = (PV+Wind+FC) + Batt - Load - Elec - Loss;   % gen+stor - load - cons - loss
    prow('BAL residual',     resid);
    fprintf('  (BAL residual = SUMgen + storage-discharge - Load - consumption - Loss; target ~0)\n');

    %% ── independent VSC cross-check (DC injection >= VSC >= Load chain) ──
    fprintf('\n  Independent cross-check (measured VSC transfer, MW):\n');
    DCinj = PV+Wind+FC+Batt-Elec;
    fprintf('  %-14s %10s %10s %10s\n', '', lbl{1}, lbl{2}, lbl{3});
    prow('DC injection', DCinj);
    prow('VSC (DC->AC)', VSC);
    prow('Load',         Load);
    prow('Loss DC-side', Lossdc);
    prow('Loss AC-side', Lossac);
    fprintf('  (expect DC injection >= VSC >= Load; Loss = DC-side + AC-side)\n');

    %% ── context ──
    fprintf('\n  context:  V_dc mean [%.1f %.1f %.1f] V | f mean [%.4f %.4f %.4f] Hz\n', ...
        Vdc(1),Vdc(2),Vdc(3), f(1),f(2),f(3));
    fprintf('==========================================================================\n');

    %% ── assemble a MATLAB table + write text ──
    T = table([-Load(:) PV(:) Wind(:) Batt(:) FC(:) -Elec(:) VSC(:) -Loss(:)].', ...
        'RowNames',{'Load','PV','Wind','Battery','FC','Electrolyzer','VSC','Loss'});
    T.Properties.VariableNames = {'MW'};
    % (the numeric grid itself is the more useful artifact; save it explicitly)
    grid = [-Load; PV; Wind; Batt; FC; -Elec; VSC; -Loss];  % 8x3
    rowN = {'Load','PV','Wind','Battery','FC','Electrolyzer','VSC','Loss'};
    verDir = local_verdir();
    txt = fullfile(verDir,'POWER_BALANCE.txt');
    fid = fopen(txt,'w');
    if fid>0
        fprintf(fid,'ISLAND POWER BALANCE (last 5 s mean, MW) — gen/discharge +, load/consumption -\n');
        fprintf(fid,'%-14s %10s %10s %10s\n','Row',lbl{:});
        for i=1:8, fprintf(fid,'%-14s %+10.2f %+10.2f %+10.2f\n',rowN{i},grid(i,:)); end
        fprintf(fid,'%-14s %+10.3f %+10.3f %+10.3f\n','BAL residual',resid);
        fprintf(fid,'\ncross-check (MW):\n');
        fprintf(fid,'%-14s %+10.2f %+10.2f %+10.2f\n','DC injection',DCinj);
        fprintf(fid,'%-14s %+10.2f %+10.2f %+10.2f\n','VSC',VSC);
        fprintf(fid,'%-14s %+10.2f %+10.2f %+10.2f\n','Loss DC-side',Lossdc);
        fprintf(fid,'%-14s %+10.2f %+10.2f %+10.2f\n','Loss AC-side',Lossac);
        fclose(fid);
        fprintf('[powerbalance] wrote %s\n', txt);
    end
    T = grid;  % return the numeric 8x3 grid (rows=rowN, cols=T1/T2/T3)
end

%% ── helpers ──
function v = gv(r, f)
    if isfield(r,f) && ~isempty(r.(f)), v = r.(f); else, v = NaN; end
end
function prow(name, v)
    fprintf('  %-14s %+10.2f %+10.2f %+10.2f\n', name, v(1), v(2), v(3));
end
function verDir = local_verdir()
    w = which('build_microgrid_eelib');
    if ~isempty(w), root = fileparts(fileparts(w));
    else, root = pwd; end
    verDir = fullfile(root,'RESULTS','verification');
    if ~exist(verDir,'dir'), mkdir(verDir); end
end
