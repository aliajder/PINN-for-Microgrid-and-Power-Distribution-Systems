function setup14Paths()
%SETUP14PATHS  ieee14bus/matlab, ieee14bus/params ve 5-bara matlab (estimateHD_LS vb.) yola ekler.
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..'));
addpath(fullfile(here, '..', 'params'));
addpath(fullfile(here, '..', '..', 'matlab'));
addpath(fullfile(here, '..', '..', '..', 'PINN-for-Synchronous', 'matlab'));
end
