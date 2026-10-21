function results = run_all_tests(includePinn)
%RUN_ALL_TESTS  Bagimsiz testleri calistirir, outputs/test_report.csv yazar.
if nargin < 1, includePinn = true; end
here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here, 'tests')); setupGeneratorPaths();
names = {'test_inputs_consistency','test_equilibrium','test_damping_form','test_ls_grid', ...
    'test_disturbance_types','test_noise','test_identifiability','test_der_independence', ...
    'test_csv_roundtrip'};
if includePinn, names{end+1} = 'test_pinn'; end
results = cell(numel(names), 4);
for i = 1:numel(names)
    tic;
    try
        [p, m] = feval(names{i});
    catch err
        p = false; m = ['HATA: ' err.message];
    end
    results(i,:) = {names{i}, p, toc, m};
    fprintf('%-28s %s  (%.0fs)  %s\n', names{i}, ternary(p), toc, m);
end
outDir = fullfile(here, '..', 'outputs'); if ~exist(outDir, 'dir'), mkdir(outDir); end
fid = fopen(fullfile(outDir, 'test_report.csv'), 'w');
fprintf(fid, 'test,pass,seconds,detail\n');
for i = 1:size(results,1)
    fprintf(fid, '%s,%d,%.1f,"%s"\n', results{i,1}, results{i,2}, results{i,3}, strrep(results{i,4}, '"', ''''));
end
fclose(fid);
fprintf('%d/%d test gecti.\n', sum([results{:,2}]), size(results,1));
end
function s = ternary(p)
if p, s = 'PASS'; else, s = 'FAIL'; end
end
