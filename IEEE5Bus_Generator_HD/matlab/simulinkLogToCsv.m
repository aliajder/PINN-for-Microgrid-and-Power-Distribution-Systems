function csvPath = simulinkLogToCsv(t, Pm_MW, Pe_MW, f_Hz, S_base_MW, f0, csvPath)
%SIMULINKLOGTOCSV  microgrid_case5.slx'teki ClassicalSM cikislarini PINN girdisine cevirir.
%   IO_CONTRACTS.md 3.2: In_Pm_MW, Out_Pe_MW, Out_f_Hz portlari kaydedilmelidir.
%   Tm = Pm/S_base, Te = Pe/S_base, w = f/f0 (makine bazinda pu; model Pm-Pe'yi
%   dogrudan guc olarak kullandigi icin ayrica w'ya bolunmez).
t = t(:); Tm = Pm_MW(:)/S_base_MW; Te = Pe_MW(:)/S_base_MW; w = f_Hz(:)/f0;
fid = fopen(csvPath, 'w');
fprintf(fid, 't,Tm,Te,w\n');
for i = 1:numel(t)
    fprintf(fid, '%.6f,%.9f,%.9f,%.9f\n', t(i), Tm(i), Te(i), w(i));
end
fclose(fid);
end
