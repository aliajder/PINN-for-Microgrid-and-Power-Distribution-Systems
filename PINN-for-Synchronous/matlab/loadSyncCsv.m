function [t, Y, trueH, trueD] = loadSyncCsv(path, st, et)
%LOADSYNCCSV Read RTDS sync-machine CSV (both column layouts).
if nargin < 2, st = 0; end
if nargin < 3, et = 1; end

fid = fopen(path, 'r');
if fid < 0
    error('Cannot open %s', path);
end
header = fgetl(fid);
fclose(fid);
header = strrep(header, '"', '');
parts = strsplit(header, ',');
shorts = cell(size(parts));
for i = 1:numel(parts)
    tok = strsplit(strtrim(parts{i}), '|');
    shorts{i} = tok{end};
end

timeCol = pickColumn(shorts, {'Time'});
tmCol = pickColumn(shorts, {'Tm'});
wCol = pickColumn(shorts, {'w_sync', 'WPU', 'TESTSPDOUT', 'w'});
teCol = pickColumn(shorts, {'Te_sync', 'TESTTELECT', 'Te'});
if any(isnan([timeCol, tmCol, wCol, teCol]))
    error('Missing Time/Tm/w/Te columns in %s', path);
end

data = dlmread(path, ',', 1, 0);
tRaw = data(:, timeCol);
Yraw = [data(:, tmCol), data(:, wCol), data(:, teCol)];
n = size(Yraw, 1);
stepTime = tRaw(2) - tRaw(1);
i0 = max(1, floor(n * st) + 1);
i1 = min(n, max(i0, floor(n * et)));
t = tRaw(i0:i1) - stepTime * floor(n * st);
Y = Yraw(i0:i1, :);
[trueH, trueD] = parseTrueHD(path);
end

function idx = pickColumn(shorts, names)
idx = NaN;
for k = 1:numel(names)
    for j = 1:numel(shorts)
        if strcmp(shorts{j}, names{k})
            idx = j;
            return
        end
    end
end
end
