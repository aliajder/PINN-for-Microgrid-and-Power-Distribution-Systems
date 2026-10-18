function setupGeneratorPaths()
%SETUPGENERATORPATHS  Bu klasoru ve trainSwingPINN.m klasorunu yola ekler.
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', '..', 'PINN-for-Synchronous', 'matlab'));
end
