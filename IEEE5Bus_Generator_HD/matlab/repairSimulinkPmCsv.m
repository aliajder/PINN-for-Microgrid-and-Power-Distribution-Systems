function repairSimulinkPmCsv(csvPath, Sbase, pm0, frac, t0, rampt)
%REPAIRSIMULINKPMCSV  Eski kayitta Tm'nin Pmech_MW (sabit) kalmasi hatasini duzeltir.
%   Bus 3, +%8 rampa, Sbase=520, pm0=323.49 icin:
%     repairSimulinkPmCsv('../outputs/data/Simulink_SM_Bus3.csv', 520, 323.49, 0.08, 1, 0.1)
%     estimateGeneratorFromCsv('../outputs/data/Simulink_SM_Bus3.csv')
if nargin < 6, rampt = 0.1; end
[t, Tm, Te, w] = loadGeneratorCsv(csvPath); %#ok<ASGLU>
u = min(max((t - t0) / rampt, 0), 1);
Pm = pm0 * (1 + frac * u);
Tm = Pm / Sbase;
fid = fopen(csvPath, 'w');
fprintf(fid, 't,Tm,Te,w\n');
for i = 1:numel(t)
    fprintf(fid, '%.6f,%.9f,%.9f,%.9f\n', t(i), Tm(i), Te(i), w(i));
end
fclose(fid);
fprintf('Tm duzeltildi (Pm %.4f -> %.4f MW): %s\n', pm0, pm0*(1+frac), csvPath);
end
