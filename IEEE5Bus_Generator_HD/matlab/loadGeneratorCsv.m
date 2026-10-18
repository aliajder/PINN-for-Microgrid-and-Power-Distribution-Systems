function [t, Tm, Te, w] = loadGeneratorCsv(csvPath)
%LOADGENERATORCSV  t,Tm,Te,w sutunlu CSV'yi okur (Octave ve MATLAB).
fid = fopen(csvPath, 'r'); fgetl(fid);
M = fscanf(fid, '%f,%f,%f,%f', [4 Inf])'; fclose(fid);
t = M(:,1); Tm = M(:,2); Te = M(:,3); w = M(:,4);
end
