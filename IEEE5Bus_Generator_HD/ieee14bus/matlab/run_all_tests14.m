function results = run_all_tests14()
%RUN_ALL_TESTS14  14 baralı sistem testlerini kosar, ieee14bus/outputs/test_report14.csv yazar.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'tests')); setup14Paths();
names = {'test14_data','test14_equilibrium','test14_paper_event','test14_damping_form', ...
    'test14_ls_all_machines','test14_noise'};
results = cell(numel(names), 4);
for i = 1:numel(names)
    tic;
    try, [p, m] = feval(names{i}); catch err, p = false; m = ['HATA: ' err.message]; end
    results(i,:) = {names{i}, p, toc, m};
    if p, s = 'PASS'; else, s = 'FAIL'; end
    fprintf('%-26s %s  (%.0fs)  %s\n', names{i}, s, toc, m);
end
outDir = fullfile(here, '..', 'outputs'); if ~exist(outDir, 'dir'), mkdir(outDir); end
fid = fopen(fullfile(outDir, 'test_report14.csv'), 'w');
fprintf(fid, 'test,pass,seconds,detail\n');
for i = 1:size(results,1)
    fprintf(fid, '%s,%d,%.1f,"%s"\n', results{i,1}, results{i,2}, results{i,3}, strrep(results{i,4}, '"', ''''));
end
fclose(fid);
fprintf('%d/%d test gecti.\n', sum([results{:,2}]), size(results,1));
end
