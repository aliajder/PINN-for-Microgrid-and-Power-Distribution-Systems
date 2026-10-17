function [trueH, trueD] = parseTrueHD(path)
[~, name, ~] = fileparts(path);
tok = regexp(name, 'H([0-9]+(?:\.[0-9]+)?)D([0-9]+(?:\.[0-9]+)?)', 'tokens', 'once', 'ignorecase');
if isempty(tok)
    trueH = NaN;
    trueD = NaN;
else
    trueH = str2double(tok{1});
    trueD = str2double(tok{2});
end
end
