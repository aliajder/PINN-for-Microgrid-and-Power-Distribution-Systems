function build_wind(mdl)
%% build_wind.m  (Task 5.8)
% Builds the Wind subsystem (physical Cp(λ,β)+four-region turbine + averaged
% generator/rectifier) inside the master model. Feeds the common DC bus.
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.8 (Wind), CLAUDE.md A4/R5
% Params    : DATA/wind_params.mat (struct `wd`, build_wind_params.m, Rule 8)
% Physics   : PARAMS/cp_lambda.m (Cp(λ,β), used offline to get Cp_max)
%
% WHAT THIS MODULE IS (hybrid, Task 5.8 — mirrors build_pv.m):
%   RAW wind speed v [m/s] drives a physical turbine: at optimal tip-speed
%   ratio (β=0) the cube region delivers Cp_max·½ρAv³; a four-region power
%   curve caps it at P_rated (pitch) and stops outside [cut_in, cut_out].
%   An averaged generator/rectifier injects the power into the 1500 V bus as
%   ONE Controlled Current Source. ninja `electricity` is validation-only.
%
%      In_v_wind_ms ─► [wind_turbine] P_turb,Cp ─► [dcdc_ctrl] I_cmd ─►(delay)─► CCS ─► Elec_DC_out
%                                                  (I = P_turb·η_gen·η_wind / V_dc)
%
% ── DESIGN DECISIONS (2026-06-20, inherits PV P1/P2/P3) ───────────────────
%  W1. Signal-level physics + ONE Controlled Current Source on the bus
%      (PV P1 / ESS H1). No physical rotor/generator nodes at averaged scope.
%  W2. QUASI-STATIC optimal-λ (no rotor-inertia / pitch-controller dynamics):
%      Cp_max (from cp_lambda at λ_opt, β=0) is the cube-region coefficient;
%      above v_rated the turbine pitches, modelled as P held at P_rated with an
%      effective Cp = P_rated/(½ρAv³) < Cp_max (reported for the scope). This
%      is the wind analogue of PV's MPP-at-optimum and is adequate at R5 scope.
%      → only ONE algebraic-loop Unit Delay needed (the DC-side I=P/V_dc), vs
%      PV's two (PV also closes a V_ref↔pv_cell MPPT loop; wind_turbine(v) is
%      feed-forward).
%  W3. Inports, not internal From Workspace (PV P2): In_v_wind_ms driven by the
%      test (constants) or the master (From Workspace `wind_data`).

% ── Parameters (Rule 8) ────────────────────────────────────────────────────
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
wd   = load(fullfile(root,'DATA','wind_params.mat')).wd;
dt_s = str2double(get_param(mdl,'FixedStep'));

% ── Subsystem container ───────────────────────────────────────────────────
ssName = 'Wind';  ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 670 360 810]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY PORTS
% =====================================================================
dcPort = [ssPath '/Elec_DC_out'];
add_block('built-in/PMIOPort', dcPort, 'Side', 'Left', 'Position', [40 300 60 320]);
add_block('built-in/Inport',  [ssPath '/In_v_wind_ms'], 'Position', [40 90 60 108]);
add_block('built-in/Outport', [ssPath '/Out_P_wind_MW'], 'Position', [1100 80 1120 98]);
add_block('built-in/Outport', [ssPath '/Out_v_wind_ms'], 'Position', [1100 130 1120 148]);
add_block('built-in/Outport', [ssPath '/Out_Cp'],        'Position', [1100 180 1120 198]);
add_line(ssPath, 'In_v_wind_ms/1', 'Out_v_wind_ms/1', 'autorouting','on');   % W3 pass-through

% =====================================================================
%   V_dc SENSOR (parallel at the terminal — ESS H2 / PV pattern)
% =====================================================================
add_block('ee_lib/Connectors & References/Electrical Reference', [ssPath '/Gnd'], 'Position', [340 460 380 490]);
vs = [ssPath '/V_dc_sens'];
add_block('ee_lib/Sensors & Transducers/Voltage Sensor', vs, 'Position', [180 360 240 420]);
add_line(ssPath, conn(dcPort,'RConn',1), conn(vs,'LConn',1));
add_line(ssPath, conn(vs,'RConn',2), conn([ssPath '/Gnd'],'LConn',1));
add_block('nesl_utility/PS-Simulink Converter', [ssPath '/Vdc_PS2S'], 'Position', [180 480 220 510]);
set_param([ssPath '/Vdc_PS2S'], 'Unit', 'V');
add_line(ssPath, conn(vs,'RConn',1), conn([ssPath '/Vdc_PS2S'],'LConn',1));

% =====================================================================
%   wind_turbine — four-region power curve (params injected as literals)
% =====================================================================
turb = [ssPath '/wind_turbine'];
add_block('simulink/User-Defined Functions/MATLAB Function', turb, 'Position', [320 70 500 180]);
set_mlfcn(turb, { ...
 'function [P_turb_W, Cp] = wind_turbine(v)'
 '%#codegen'
 '% Four-region curve. Cube region runs at Cp_max (optimal λ, β=0); above'
 '% v_rated the turbine pitches (P held at P_rated, effective Cp falls).'
 ['Cpmax = ' num2str(wd.Cp_max,'%.6g') ';  rho = ' num2str(wd.rho) ';  A = ' num2str(wd.A,'%.8g') ';']
 ['cin = ' num2str(wd.cut_in) ';  vr = ' num2str(wd.v_rated) ';  cout = ' num2str(wd.cut_out) ';  Prated = ' num2str(wd.P_rated_W,'%.6g') ';']
 '    if v < cin || v >= cout'
 '        P_turb_W = 0; Cp = 0; return;     % below cut-in or past cut-out'
 '    end'
 '    Pwind = 0.5*rho*A*v^3;                % power in the wind through area A'
 '    Pmech = Cpmax * Pwind;                % cube region at optimal lambda'
 '    if Pmech > Prated'
 '        P_turb_W = Prated;                % pitch-regulated plateau'
 '        Cp = Prated / Pwind;              % effective Cp (< Cp_max)'
 '    else'
 '        P_turb_W = Pmech;                 % cube region'
 '        Cp = Cpmax;'
 '    end'
 'end'});
setDiscrete(turb, dt_s);
add_line(ssPath, 'In_v_wind_ms/1', 'wind_turbine/1', 'autorouting','on');
add_line(ssPath, 'wind_turbine/2', 'Out_Cp/1', 'autorouting','on');

% =====================================================================
%   dcdc_ctrl — averaged generator+rectifier → bus current command
% =====================================================================
dcdc = [ssPath '/dcdc_ctrl'];
add_block('simulink/User-Defined Functions/MATLAB Function', dcdc, 'Position', [560 260 720 340]);
set_mlfcn(dcdc, { ...
 'function [I_cmd_A, P_bus_MW] = dcdc_ctrl(P_turb_W, V_dc_V)'
 '%#codegen'
 '% Averaged generator+rectifier+DC-DC: deliver P_turb·η to the bus as I=P/V.'
 ['eta = ' num2str(wd.eta_gen*wd.eta_wind,'%.6g') ';  Vfloor = ' num2str(wd.V_floor) ';']
 '    P_bus_W  = P_turb_W * eta;'
 '    I_cmd_A  = P_bus_W / max(V_dc_V, Vfloor);'
 '    P_bus_MW = P_bus_W / 1e6;'
 'end'});
setDiscrete(dcdc, dt_s);
add_line(ssPath, 'wind_turbine/1', 'dcdc_ctrl/1', 'autorouting','on');
add_line(ssPath, 'Vdc_PS2S/1',     'dcdc_ctrl/2', 'autorouting','on');
add_line(ssPath, 'dcdc_ctrl/2',    'Out_P_wind_MW/1', 'autorouting','on');

% =====================================================================
%   CONTROLLED CURRENT SOURCE on the + rail (W1; PV/ESS CCS pattern)
%   I_cmd → Unit Delay (breaks V_dc↔I=P/V_dc↔network loop) → S-PS → CCS.
% =====================================================================
add_block('simulink/Discrete/Unit Delay', [ssPath '/I_delay'], ...
    'SampleTime', num2str(dt_s), 'Position', [760 285 790 315]);
add_block('nesl_utility/Simulink-PS Converter', [ssPath '/I_S2PS'], 'Position', [820 285 860 315]);
set_param([ssPath '/I_S2PS'], 'Unit', 'A', 'FilteringAndDerivatives', 'filter', ...
    'SimscapeFilterOrder', '1', 'InputFilterTimeConstant', '1e-4');
add_line(ssPath, 'dcdc_ctrl/1', 'I_delay/1', 'autorouting','on');
add_line(ssPath, 'I_delay/1',   'I_S2PS/1',  'autorouting','on');

ccs = [ssPath '/Wind_CCS'];
add_block('fl_lib/Electrical/Electrical Sources/Controlled Current Source', ccs, 'Position', [520 360 580 440]);
add_line(ssPath, conn([ssPath '/I_S2PS'],'RConn',1), conn(ccs,'RConn',1));   % PS cmd
add_line(ssPath, conn(ccs,'LConn',1), conn(dcPort,'RConn',1));               % + rail
add_line(ssPath, conn(ccs,'RConn',2), conn([ssPath '/Gnd'],'LConn',1));      % return (F1)

fprintf('  build_wind: 120 MW turbine, Cp_max=%.2f, 4-region (cut_in %.0f/rated %.0f/cut_out %.0f), eta=%.2f; feeds Elec_Wind_term.\n', ...
    wd.Cp_max, wd.cut_in, wd.v_rated, wd.cut_out, wd.eta_gen*wd.eta_wind);
end % build_wind


% =====================================================================
%                              HELPERS
% =====================================================================
function h = conn(blkPath, side, idx)
ph = get_param(blkPath, 'PortHandles');
h  = ph.(side)(idx);
end

function set_mlfcn(blkPath, lines)
cfg = get_param(blkPath, 'MATLABFunctionConfiguration');
cfg.FunctionScript = strjoin(lines, newline);
end

function setDiscrete(blkPath, dt_s)
cfg = get_param(blkPath, 'MATLABFunctionConfiguration');
cfg.UpdateMethod = 'Discrete';
cfg.SampleTime   = num2str(dt_s);
end
