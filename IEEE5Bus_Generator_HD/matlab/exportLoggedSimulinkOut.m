function result = exportLoggedSimulinkOut(simOut, opts)
%EXPORTLOGGEDSIMULINKOUT  Zaten kosulmus `out` (SimulationOutput) -> CSV + LS.
%   Simulasyonu tekrar kosmadan, Command Window'daki `out` ile:
%       exportLoggedSimulinkOut(out)
%       exportLoggedSimulinkOut(out, struct('gitCommit', true, 'gitPush', true))
if nargin < 1 || isempty(simOut)
    if evalin('base', 'exist(''out'',''var'')')
        simOut = evalin('base', 'out');
    else
        error('out yok. Once simulasyonu kosun veya simOut verin.');
    end
end
if nargin < 2, opts = struct(); end
if ~isfield(opts, 'Sbase'), opts.Sbase = []; end
if ~isfield(opts, 'f0'), opts.f0 = 50; end
if ~isfield(opts, 'pinn'), opts.pinn = false; end
if ~isfield(opts, 'gitCommit'), opts.gitCommit = true; end
if ~isfield(opts, 'gitPush'), opts.gitPush = false; end

here = fileparts(mfilename('fullpath'));
setupGeneratorPaths();
root = fileparts(here);
dataDir = fullfile(root, 'outputs', 'data');
if exist(dataDir, 'dir') ~= 7, mkdir(dataDir); end

buses = [1 3 5];
if isempty(opts.Sbase)
    opts.Sbase = [210 520 600];
    warning('Sbase verilmedi, [210 520 600] MW varsayildi. SM_Bus*/Sbase_MW ile dogrulayin.');
end
fid = fopen(fullfile(root, 'outputs', 'simulink_HD_estimates.csv'), 'w');
fprintf(fid, 'bus,Sbase_MW,wSpan,identifiable,LS_H,LS_D,c,csv\n');
result.csv = cell(1, 3);
for k = 1:3
    b = buses(k);
    Pm = getts(simOut, sprintf('Pm_MW_b%d', b));
    Pe = getts(simOut, sprintf('Pe_MW_b%d', b));
    w  = getts(simOut, sprintf('w_pu_b%d', b));
    csv = fullfile(dataDir, sprintf('Simulink_SM_Bus%d.csv', b));
    simulinkLogToCsv(Pm.Time(:), Pm.Data(:), Pe.Data(:), opts.f0*w.Data(:), opts.Sbase(k), opts.f0, csv);
    est = estimateGeneratorFromCsv(csv, struct('pinn', opts.pinn, 'dampingRef', 1));
    result.csv{k} = csv;
    result.est(k) = est; %#ok<AGROW>
    fprintf(fid, '%d,%.6f,%.6e,%d,%.6f,%.6f,%.6g,%s\n', ...
        b, opts.Sbase(k), est.ls.wSpan, est.ls.identifiable, est.ls.H, est.ls.D, est.free.c, csv);
end
fclose(fid);
if opts.gitCommit
    repo = fileparts(root);
    cmd = @(c) system(sprintf('git -C "%s" %s', repo, c));
    cmd('add IEEE5Bus_Generator_HD/outputs/data/Simulink_SM_Bus*.csv IEEE5Bus_Generator_HD/outputs/simulink_HD_estimates.csv');
    [st, ~] = cmd('diff --cached --quiet');
    if st ~= 0
        cmd(sprintf('commit -m "Add Simulink SM_Bus recordings (%s)"', datestr(now, 31)));
        fprintf('Git commit yapildi.\n');
        if opts.gitPush
            [stp, outp] = cmd('push');
            fprintf('%s', outp);
            if stp ~= 0, warning('git push basarisiz; klasorden git push calistirin.'); end
        end
    else
        fprintf('Git: commitlenecek CSV yok.\n');
    end
end
end

function ts = getts(simOut, name)
if isa(simOut, 'Simulink.SimulationOutput')
    try, ts = simOut.get(name); catch, ts = simOut.(name); end
else
    ts = simOut.(name);
end
end
