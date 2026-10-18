function [matlabDir, dataDir] = setupSwingPinnPaths()
%SETUPSWINGPINNPATHS Locate helper .m files and RTDS CSVs, then addpath.
%
% Works when the project lives in the git repo:
%   <repo>/PINN-for-Synchronous/matlab/*.m
%   <repo>/data/synchronous-machine/*.csv
% or when everything is copied next to the Live Script:
%   C:\Users\<name>\Documents\matlab\*.m
%   C:\Users\<name>\Documents\matlab\data\synchronous-machine\*.csv
%
% Live Editor often makes mfilename('fullpath') of the .mlx/.m script point
% at a temporary folder. Locating loadSyncCsv.m from THIS function is reliable.

csvName = '1208_SyncOn_H1.5D0.25.csv';
cands = uniqueCandidates();

matlabDir = firstExistingFile(cands, 'loadSyncCsv.m');
if isempty(matlabDir)
    w = which('loadSyncCsv');
    if ~isempty(w)
        matlabDir = fileparts(w);
    end
end
if isempty(matlabDir)
    error(['loadSyncCsv.m bulunamadi. PINN_Salinim_D_Tahmini.m ile ayni klasore ' ...
        'loadSyncCsv.m, trainSwingPINN.m ve setupSwingPinnPaths.m dosyalarini koyun.']);
end
addpath(matlabDir);

dataRoots = cands;
dataRoots{end + 1} = matlabDir;
dataRoots{end + 1} = fileparts(matlabDir);
dataRoots{end + 1} = fileparts(fileparts(matlabDir));
dataDirs = {};
for i = 1:numel(dataRoots)
    dataDirs{end + 1} = fullfile(dataRoots{i}, 'data', 'synchronous-machine'); %#ok<AGROW>
    dataDirs{end + 1} = fullfile(dataRoots{i}, 'synchronous-machine'); %#ok<AGROW>
end
dataDir = firstExistingFile(dataDirs, csvName);
if isempty(dataDir)
    error(['Ornek CSV bulunamadi (%s). Beklenen klasor: %s'], ...
        csvName, fullfile(matlabDir, 'data', 'synchronous-machine'));
end
end

function cands = uniqueCandidates()
cands = {};
try
    active = matlab.desktop.editor.getActiveFilename;
    cands = appendIfNonempty(cands, fileparts(active));
catch
end
here = fileparts(mfilename('fullpath'));
cands = appendIfNonempty(cands, here);
cands = appendIfNonempty(cands, pwd);
if exist('userpath', 'file') == 2 || exist('userpath', 'builtin')
    up = userpath;
    if ischar(up) || isstring(up)
        up = char(up);
        up = strrep(up, pathsep, '');
        cands = appendIfNonempty(cands, strtrim(up));
    end
end
home = getenv('USERPROFILE');
if isempty(home)
    home = getenv('HOME');
end
if ~isempty(home)
    cands = appendIfNonempty(cands, fullfile(home, 'Documents', 'matlab'));
    cands = appendIfNonempty(cands, fullfile(home, 'Documents', 'MATLAB'));
end
keep = {};
for i = 1:numel(cands)
    p = cands{i};
    seen = false;
    for j = 1:numel(keep)
        if strcmpi(keep{j}, p)
            seen = true;
            break
        end
    end
    if ~seen
        keep{end + 1} = p; %#ok<AGROW>
    end
end
cands = keep;
end

function cands = appendIfNonempty(cands, p)
if ~isempty(p)
    cands{end + 1} = p;
end
end

function folder = firstExistingFile(folders, fileName)
folder = '';
for i = 1:numel(folders)
    if isempty(folders{i})
        continue
    end
    if exist(fullfile(folders{i}, fileName), 'file') == 2
        folder = folders{i};
        return
    end
end
end
