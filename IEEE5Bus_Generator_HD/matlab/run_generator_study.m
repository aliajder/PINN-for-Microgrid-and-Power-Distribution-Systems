function T = run_generator_study(usePinn)
%RUN_GENERATOR_STUDY  IEEE 5-bus jeneratorlerinin (Bus 1,3,5) H ve D'sini bulur.
%   Simulink'siz case5SwingSim ile veri uretilir (build_transmission.m ile ayni
%   ClassicalSM denklemi, H=5.5 s, D=1.5 pu); LS ve PINN ile geri bulunur.
%   Cikti: outputs/generator_HD_estimates.csv, outputs/data/*.csv, outputs/*.png
if nargin < 1, usePinn = true; end
here = fileparts(mfilename('fullpath')); addpath(here); setupGeneratorPaths();
outDir = fullfile(here, '..', 'outputs'); dataDir = fullfile(outDir, 'data');
if ~exist(dataDir, 'dir'), mkdir(dataDir); end
busName = [1 3 5];
rows = {}; Tend = 10;
fprintf('%-26s %-7s %-6s %8s %8s %8s %8s\n', 'senaryo', 'makine', 'yontem', 'H', 'D', 'H_hata%', 'D_hata%');
for k = 1:3
    sc = sprintf('PmStep20_Bus%d', busName(k));
    o = case5SwingSim(struct('Tend', Tend, 'pmFun', @(t) 1 + 0.2*(t >= 1)*double((1:3) == k)));
    csv = fullfile(dataDir, [sc '.csv']); exportGeneratorCsv(o, k, csv);
    rows = [rows; addRow(sc, busName(k), 'LS', estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 1), o, k)]; %#ok<AGROW>
    if usePinn
        rng(3); p = estimateHD_PINN(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k));
        rows = [rows; addRow(sc, busName(k), 'PINN', p, o, k)]; %#ok<AGROW>
    end
    if k == 2, oPlot = o; end
end
sc = 'LoadStep100MW_Bus3';
SL = @(t) complex([0;0;300+100*(t>=1);400;0], [0;0;98.61+32.9*(t>=1);131.47;0]);
o = case5SwingSim(struct('Tend', Tend, 'loadFun', SL));
for k = 1:3
    exportGeneratorCsv(o, k, fullfile(dataDir, sprintf('%s_machine_Bus%d.csv', sc, busName(k))));
    rows = [rows; addRow(sc, busName(k), 'LS', estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 1), o, k)]; %#ok<AGROW>
end
fid = fopen(fullfile(outDir, 'generator_HD_estimates.csv'), 'w');
fprintf(fid, 'scenario,bus,method,true_H,true_D,est_H,est_D,H_err_pct,D_err_pct\n');
for i = 1:size(rows,1)
    r = rows(i,:);
    fprintf(fid, '%s,%d,%s,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f\n', r{:});
    fprintf('%-26s Bus%-4d %-6s %8.4f %8.4f %8.2f %8.2f\n', r{1}, r{2}, r{3}, r{6}, r{7}, r{8}, r{9});
end
fclose(fid);
T = rows;
try
    makeFigure(oPlot, outDir);
catch err
    fprintf('Sekil atlandi (%s). outputs/plot_outputs.py ile cizilebilir.\n', err.message);
end
end

function row = addRow(sc, bus, method, r, o, k)
row = {sc, bus, method, o.H(k), o.D(k), r.H, r.D, 100*(r.H-o.H(k))/o.H(k), 100*(r.D-o.D(k))/o.D(k)};
end

function makeFigure(o, outDir)
k = 2;
try, graphics_toolkit('gnuplot'); catch, end
fh = figure('visible', 'off');
subplot(2,1,1); plot(o.t, o.Tm(:,k), o.t, o.Te(:,k), 'LineWidth', 1.5);
legend('Tm', 'Te'); ylabel('pu'); title('Bus 3 jeneratoru: Pm basamagi (t=1 s)'); grid on;
subplot(2,1,2); plot(o.t, o.w(:,k) - 1, 'LineWidth', 1.5);
ylabel('\omega - 1 [pu]'); xlabel('t [s]'); grid on;
print(fh, fullfile(outDir, 'generator_bus3_response.png'), '-dpng');
close(fh);
end
