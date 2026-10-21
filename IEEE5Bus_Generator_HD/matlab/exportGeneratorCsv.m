function csvPath = exportGeneratorCsv(out, k, csvPath)
%EXPORTGENERATORCSV  case5SwingSim ciktisindan makine k icin t,Tm,Te,w CSV yazar.
fid = fopen(csvPath, 'w');
fprintf(fid, 't,Tm,Te,w\n');
for i = 1:numel(out.t)
    fprintf(fid, '%.6f,%.9f,%.9f,%.9f\n', out.t(i), out.Tm(i,k), out.Te(i,k), out.w(i,k));
end
fclose(fid);
end
