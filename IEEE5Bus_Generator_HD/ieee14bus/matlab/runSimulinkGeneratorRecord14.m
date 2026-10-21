function result = runSimulinkGeneratorRecord14(opts)
%RUNSIMULINKGENERATORRECORD14  microgrid_case14.slx: 5 makine icin Pm/Pe/w kaydi + LS.
%   Onkosul: ieee14bus/SIMULINK/rebuild_microgrid_case14.m bir kez kosulmus olmali.
%   Pm basamagi: ünite 16 (bara 16, 100 MVA), +15 MW, t=1 s, 1.25 s rampa.
%   Cikti: ieee14bus/outputs/data/IEEE14_Simulink_Unit{15,16,17,19,18}.csv
%          ieee14bus/outputs/ieee14_simulink_HD_estimates.csv
%   Not: D=0 olan kondansatorlerde (17,19,18) LS D tahmini ~0 beklenir.
if nargin < 1, opts = struct(); end
here   = fileparts(mfilename('fullpath'));
ieee14 = fileparts(here);
repoIn = fullfile(fileparts(ieee14), 'matlab');
addpath(repoIn);
d.slx = fullfile(ieee14, 'SIMULINK', 'microgrid_case14.slx');
d.buses = [15 16 17 19 18];
d.pmBus = 16;
d.pmStepMW = 15;
d.rampSec = 1.25;
d.outRoot = ieee14;
d.csvPrefix = 'IEEE14_Simulink_Unit';
d.summaryName = 'ieee14_simulink_HD_estimates.csv';
d.extraParams = {fullfile(ieee14, 'mg14_params.mat'), 'mg14'};
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
if exist(opts.slx, 'file') ~= 2
    error('Once rebuild_microgrid_case14 kosun (model yok: %s)', opts.slx);
end
result = runSimulinkGeneratorRecord(opts);
end
