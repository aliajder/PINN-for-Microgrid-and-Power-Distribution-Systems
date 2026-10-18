%% vrun_powerbalance.m — run T1/T2/T3 island cases and emit the power-balance table
% Re-runs the three island scenarios with the NEW measured power channels
% (P_pv, P_wind, P_vsc, and the derived P_loss) that were added to
% vrun_island.m / vrun_island_step.m, then assembles the publication table
% (rows {Load, PV, Wind, Battery, FC, Electrolyzer, VSC, Loss}, cols {T1,T2,T3})
% via powerbalance_table.m and runs the per-column balance check.
%
% Scenarios (identical to the previously-validated runs — physics untouched):
%   T1  island 210 MW baseline           StopTime 30 s
%   T2  island 160 MW light load         StopTime 30 s
%   T3  island 160->210 MW load step @15 s, ramp 1.5 s, StopTime 30 s
%
% RUNTIME: each full 30 s / 50 us island run is ~75 min wall (see SUMMARY.txt);
% all three ~= 3.5-4 h. Intended to be launched head-less (matlab -batch) or
% outside the MCP. Each vrun_* saves its own RESULTS/verification/<case>.mat, so
% if this driver is interrupted AFTER the sims finish, regenerate the table with
%   powerbalance_table()
%
% OUTPUT: prints the table; writes RESULTS/verification/POWER_BALANCE.txt and
%         RESULTS/verification/powerbalance_all.mat (the 3 R structs).

clear; clc;

%% ── put the verification + SIMULINK folders on the path, cd to project root ──
here = fileparts(mfilename('fullpath'));            % .../SIMULINK/verification
simDir = fileparts(here);                           % .../SIMULINK
root = fileparts(simDir);                           % .../microh2grid
if exist(fullfile(root,'mg_params.mat'),'file') ~= 2
    for cand = {pwd, fileparts(pwd)}
        if exist(fullfile(cand{1},'mg_params.mat'),'file') == 2, root = cand{1}; break; end
    end
end
addpath(simDir); addpath(here);
cd(root);
fprintf('[powerbalance] project root = %s\n', root);

%% ── run the three scenarios (full, physics runs; isSmoke=false) ──
tAll = tic;
fprintf('\n[powerbalance] === T1 island 210 MW baseline (30 s) ===\n');
R1 = vrun_island('T1_island_baseline', 210, 30, false);

fprintf('\n[powerbalance] === T2 island 160 MW light (30 s) ===\n');
R2 = vrun_island('T2_island_light', 160, 30, false);

fprintf('\n[powerbalance] === T3 island 160->210 MW step @15 s (30 s) ===\n');
R3 = vrun_island_step('T3_h2_dispatch', 160, 210, 15, 30, false, 1.5);

fprintf('\n[powerbalance] all three ran, wall %.0f s (%.1f h)\n', toc(tAll), toc(tAll)/3600);

%% ── consistency guard vs previously-validated runs (T1: batt ~54, FC ~30) ──
fprintf('\n[powerbalance] consistency check vs prior runs:\n');
if isfield(R1,'Pess_mean')
    fprintf('   T1 battery mean %.1f MW (expect ~54) | T1 FC mean %.1f MW (expect ~30)\n', ...
        R1.Pess_mean, R1.Pfc_mean);
end

%% ── save the raw structs, then build + print the table ──
verDir = fullfile(root,'RESULTS','verification');
save(fullfile(verDir,'powerbalance_all.mat'),'R1','R2','R3');
grid = powerbalance_table(R1, R2, R3);   %#ok<NASGU>

fprintf('\n[powerbalance] DONE. Table + POWER_BALANCE.txt in %s\n', verDir);
