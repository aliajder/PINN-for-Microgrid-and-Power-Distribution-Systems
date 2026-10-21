function rebuild_microgrid_case14(opts)
%REBUILD_MICROGRID_CASE14  microgrid_case5.slx -> microgrid_case14.slx
%   TransmissionNetwork = modifiye IEEE 14 bara (19 bara). Tum mikro sebeke
%   (VSC, DC bara, PV, ruzgar, ESS, EMS, H2 ...) Bara 19'a (13.8 kV) PCC
%   trafosu (13.8/33 kV) ile baglanir. Orijinal .slx DEGISMEZ.
%
%   Onkosul : MATLAB + Simscape Electrical; MATPOWER (yalniz case14_to_simscape icin;
%             mg14_params.mat hazirsa gerekmez).
%   Kullanim: >> cd IEEE5Bus_Generator_HD/ieee14bus/SIMULINK
%             >> rebuild_microgrid_case14
%   opts    : .regen (false)  true ise case14_to_simscape yeniden kosar
%             .mgLoad_MW      Src_Pload_MW (varsayilan mg14.microgrid.Pload_MW)
%             .dt (50e-6)     .solver ('ode14x')

if nargin < 1, opts = struct(); end
if ~isfield(opts,'regen'), opts.regen = false; end
if ~isfield(opts,'dt'), opts.dt = 50e-6; end
if ~isfield(opts,'solver'), opts.solver = 'ode14x'; end

here   = fileparts(mfilename('fullpath'));
ieee14 = fileparts(here);
inSim  = fullfile(fileparts(ieee14), 'inputs', 'SIMULINK');
src    = fullfile(inSim, 'microgrid_case5.slx');
dst    = fullfile(here, 'microgrid_case14.slx');
addpath(here);

if opts.regen || exist(fullfile(ieee14,'mg14_params.mat'),'file') ~= 2
    addpath(ieee14);
    mg14 = case14_to_simscape();
else
    S = load(fullfile(ieee14,'mg14_params.mat')); mg14 = S.mg14;
end
if isfield(opts,'mgLoad_MW')
    mg14.microgrid.Pload_MW = opts.mgLoad_MW;
    mg14.microgrid.Qload_MVAr = opts.mgLoad_MW * tan(acos(mg14.microgrid.pf));
end
assignin('base','mg14',mg14);
Sm = load(fullfile(fileparts(ieee14),'inputs','mg_params.mat'));
assignin('base','mg',Sm.mg);
assignin('base','f0',mg14.base.fbase);

if bdIsLoaded('microgrid_case14'), close_system('microgrid_case14',0); end
copyfile(src, dst, 'f');
fileattrib(dst, '+w');
load_system(dst);
mdl = 'microgrid_case14';
T   = [mdl '/TransmissionNetwork'];
PCC = [mdl '/PCC'];

% --- eski T <-> PCC hatti ---
hT = connByName(T, 'Elec_PCC_230kV');
ln = get_param(hT, 'Line');
if ln > 0, delete_line(ln); end

% --- yeni TransmissionNetwork ---
build_transmission14(mdl);

% --- PCC trafosu 13.8/33 kV ---
oldXf = [PCC '/XFMR_230_33'];
if getSimulinkBlockHandle(oldXf) > 0
    set_param(oldXf, 'Name', 'XFMR_13p8_33');
end
xf = [PCC '/XFMR_13p8_33'];
xl = mg14.pcc.Xleak_pu;
set_param(xf, 'SRated', num2str(mg14.pcc.S_MVA*1e6), ...
    'VRated1', num2str(mg14.pcc.V_hv_LL), 'VRated2', num2str(mg14.pcc.V_mv_LL), ...
    'pu_Xl1', num2str(xl/2), 'pu_Xl2', num2str(xl/2));
if getSimulinkBlockHandle([PCC '/Elec_HV_230kV']) > 0
    set_param([PCC '/Elec_HV_230kV'], 'Name', 'Elec_HV_13p8kV');
end

% --- baglanti: T.Elec_PCC_13p8kV <-> PCC.Elec_HV_13p8kV ---
add_line(mdl, connByName(T,'Elec_PCC_13p8kV'), connByName(PCC,'Elec_HV_13p8kV'));

% --- mikro sebeke yuk kaynaklari ---
setConst(mdl, 'Src_Pload_MW',    mg14.microgrid.Pload_MW);
setConst(mdl, 'Src_Qload_MVAr',  mg14.microgrid.Qload_MVAr);

set_param(mdl, 'SolverType','Fixed-step', 'Solver', opts.solver, ...
    'FixedStep', num2str(opts.dt,16), 'StopTime', '10');

try
    set_param(mdl, 'SimulationCommand', 'update');
    fprintf('  rebuild_microgrid_case14: model derlendi (update diagram OK).\n');
catch ME
    warning('Derleme uyarisi: %s', ME.message);
end
save_system(mdl, dst);
fprintf('  Kaydedildi: %s\n', dst);
fprintf('  PCC: bara 19 (13.8 kV) -> trafo %.0f MVA -> 33 kV mikro sebeke, yuk %.1f MW\n', ...
    mg14.pcc.S_MVA, mg14.microgrid.Pload_MW);
end

function setConst(mdl, name, val)
b = [mdl '/' name];
if getSimulinkBlockHandle(b) > 0
    set_param(b, 'Value', num2str(val));
else
    warning('%s bulunamadi (mikro sebeke yuku atlandi).', name);
end
end

function h = connByName(ss, name)
blk = [ss '/' name];
side  = get_param(blk,'Side');
mynum = str2double(get_param(blk,'Port'));
pmios = find_system(ss,'SearchDepth',1,'BlockType','PMIOPort');
nums = [];
for i=1:numel(pmios)
    if strcmp(get_param(pmios{i},'Side'), side)
        nums(end+1) = str2double(get_param(pmios{i},'Port')); %#ok<AGROW>
    end
end
nums = sort(nums);
rk = find(nums==mynum, 1);
ph = get_param(ss,'PortHandles');
if strcmpi(side,'Left'), h = ph.LConn(rk); else, h = ph.RConn(rk); end
end
