function results = runEstimateMachineD(varargin)
%RUNESTIMATEMACHINED Swing-equation PINN D estimation (MATLAB / Octave).
%
%   runEstimateMachineD
%   runEstimateMachineD('iterations', 4000)
%   runEstimateMachineD('cases', {'SyncOn_H1.0D0.0', 'SyncOn_H1.5D0.25'})
%
% From MATLAB locally: add this folder to the path, then call this function.

thisDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(thisDir));
addpath(thisDir);

opts = parseArgs(varargin, repoRoot);
if ~exist(opts.outputDir, 'dir')
    mkdir(opts.outputDir);
end

files = listMachineCsvs(opts.dataDir, opts.cases);
nFiles = numel(files);
results = cell(nFiles, 7);
outCsv = fullfile(opts.outputDir, 'D_estimates_matlab.csv');
fid = fopen(outCsv, 'w');
fprintf(fid, 'file,true_H,true_D,pred_H,pred_D,rel_error_D,log_file\n');

for i = 1:nFiles
    path = files{i};
    [~, base, ext] = fileparts(path);
    name = [base, ext];
    fprintf('Estimating D for %s\n', name);
    [t, Y, trueH, trueD] = loadSyncCsv(path, opts.st, opts.et);
    if opts.stride > 1
        t = t(1:opts.stride:end);
        Y = Y(1:opts.stride:end, :);
    end
    trainOpts = struct('iterations', opts.iterations, 'lr', opts.lr, 'period', opts.period);
    [predH, predD, hist] = trainSwingPINN(t, Y, trainOpts);
    if isnan(trueD)
        relD = NaN;
    elseif abs(trueD) < 1e-12
        relD = abs(predD - trueD);
    else
        relD = abs(predD - trueD) / abs(trueD);
    end
    logFile = fullfile(opts.outputDir, ['SYNC_variables_matlab_', base, '.dat']);
    writeHist(logFile, hist);
    relLog = ['results/', 'SYNC_variables_matlab_', base, '.dat'];
    results(i, :) = {name, trueH, trueD, predH, predD, relD, relLog};
    fprintf(fid, '%s,%.6g,%.6g,%.6g,%.6g,%.6g,%s\n', name, trueH, trueD, predH, predD, relD, relLog);
end
fclose(fid);
fprintf('Wrote %s\n', outCsv);
disp(results);
end

function files = listMachineCsvs(dataDir, cases)
listing = dir(fullfile(dataDir, '1208_*.csv'));
if isempty(listing)
    error('No 1208_*.csv files in %s', dataDir);
end
files = {};
for i = 1:numel(listing)
    name = listing(i).name;
    if isempty(cases)
        files{end + 1} = fullfile(dataDir, name); %#ok<AGROW>
    else
        for k = 1:numel(cases)
            if ~isempty(strfind(name, cases{k}))
                files{end + 1} = fullfile(dataDir, name); %#ok<AGROW>
                break
            end
        end
    end
end
files = sort(files);
if isempty(files)
    error('No cases matched the requested filters.');
end
end

function writeHist(path, hist)
fid = fopen(path, 'w');
for i = 1:size(hist, 1)
    fprintf(fid, '%d [%g, %g]\n', hist(i, 1), hist(i, 2), hist(i, 3));
end
fclose(fid);
end

function opts = parseArgs(args, repoRoot)
opts.iterations = 1000;
opts.lr = 1e-3;
opts.period = 200;
opts.st = 0;
opts.et = 1;
opts.stride = 2;
opts.cases = {};
opts.dataDir = fullfile(repoRoot, 'data', 'synchronous-machine');
opts.outputDir = fullfile(repoRoot, 'results');
i = 1;
while i <= numel(args)
    key = lower(args{i});
    if i == numel(args)
        error('Missing value for %s', key);
    end
    val = args{i + 1};
    switch key
        case 'iterations', opts.iterations = val;
        case 'lr', opts.lr = val;
        case 'period', opts.period = val;
        case 'st', opts.st = val;
        case 'et', opts.et = val;
        case 'stride', opts.stride = val;
        case 'cases'
            if ischar(val)
                opts.cases = {val};
            else
                opts.cases = val;
            end
        case 'datadir', opts.dataDir = val;
        case 'outputdir', opts.outputDir = val;
        otherwise
            error('Unknown option %s', key);
    end
    i = i + 2;
end
end
