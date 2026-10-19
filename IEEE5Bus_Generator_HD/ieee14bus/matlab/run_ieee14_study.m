function T = run_ieee14_study()
%RUN_IEEE14_STUDY  14 baralı sistemde 5 makinenin H ve D'sini LS ile bulur; CSV yazar.
%   outputs/ieee14_HD_estimates.csv, outputs/data/IEEE14_*.csv
here = fileparts(mfilename('fullpath')); setup14Paths();
outDir = fullfile(here, '..', 'outputs'); dataDir = fullfile(outDir, 'data');
if ~exist(dataDir, 'dir'), mkdir(dataDir); end
S = load(fullfile(here, '..', 'mg14_params.mat')); mg = S.mg14;
base = mg.bus.Pd_MW(1:14) + 1i*mg.bus.Qd_MVAr(1:14);
ex = zeros(14,1); ex([3 4 9]) = 0.2*base([3 4 9]);
sc = {'PaperLoadEvent', struct('Tend', 10, 'loadFun', @(t) ex*(t >= 1));
      'PmStep_Unit16',  struct('Tend', 10, 'pmFun', @(t) [1, 1 + 0.3*(t >= 1), 1, 1, 1])};
rows = {};
fprintf('%-16s %-5s %8s %8s %8s %8s\n', 'senaryo', 'unite', 'H', 'D', 'H_hata%', 'D_mutlak');
for i = 1:size(sc,1)
    o = case14SwingSim(sc{i,2});
    for k = 1:5
        csv = fullfile(dataDir, sprintf('IEEE14_%s_Unit%d.csv', sc{i,1}, o.unitBus(k)));
        fid = fopen(csv, 'w'); fprintf(fid, 't,Tm,Te,w\n');
        for j = 1:numel(o.t), fprintf(fid, '%.6f,%.9f,%.9f,%.9f\n', o.t(j), o.Tm(j,k), o.Te(j,k), o.w(j,k)); end
        fclose(fid);
        r = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 1);
        rows(end+1,:) = {sc{i,1}, o.unitBus(k), o.H(k), o.D(k), r.H, r.D, ...
            100*(r.H-o.H(k))/o.H(k), r.D - o.D(k)}; %#ok<AGROW>
        fprintf('%-16s %-5d %8.4f %8.4f %8.2f %8.4f\n', sc{i,1}, o.unitBus(k), r.H, r.D, rows{end,7}, rows{end,8});
    end
end
fid = fopen(fullfile(outDir, 'ieee14_HD_estimates.csv'), 'w');
fprintf(fid, 'scenario,unit_bus,true_H,true_D,LS_H,LS_D,H_err_pct,D_abs_err\n');
for i = 1:size(rows,1), fprintf(fid, '%s,%d,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f\n', rows{i,:}); end
fclose(fid);
T = rows;
end
