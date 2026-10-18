function build_transmission(mdl)
%% build_transmission.m
% Builds the case5 transmission network subsystem inside the master model.
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.1 (TransmissionNetwork) + §3.2 (ClassicalSM)
% Master    : SIMULINK/build_microgrid_eelib.m calls this with the model name
% Params    : read from base workspace `mg` (mg_params.mat) and `f0` (Rule 8)
%
% TOPOLOGY (case5, 230 kV):
%
%     Bus_1 ───┬───── Bus_4(slack) ───── Bus_5
%      (SM)    │         │  (ideal VS)      (SM)
%       │      │         │
%      Bus_2  ─┘        Bus_3
%      (PCC port)       (SM + 300 MW load)
%                        Bus_4: 400 MW load
%
% Branches : 1-2, 1-4, 1-5, 2-3, 3-4, 4-5  (PI sections)
% SMs      : Bus 1, 3, 5  (classical 2nd-order)
% Slack    : Bus 4 ideal Voltage Source (Three-Phase)
% PCC      : Bus 2 exposed as a Simscape connection port (PMIOPort)
% Loads    : Bus 3 (300 MW) and Bus 4 (400 MW). Bus 2 load is the microgrid
%            (external, modelled in MicrogridACBus) — NOT here.
%
% VERIFIED ee_lib PORT MAP (probed 2026-05-28):
%   Voltage Source (3ph)            : LConn(1)=N (1-phase), RConn(1)=~ (3-phase)
%   Controlled Voltage Source (3ph) : LConn(1)=PS ref in, LConn(2)=N (1-ph),
%                                     RConn(1)=~ (3-phase)
%   Phase Voltage Sensor (3ph)      : LConn(1)=~ node, RConn(1)=PS out,
%                                     RConn(2)=~ reference node
%   Current Sensor (3ph)            : LConn(1)=~ in, RConn(1)=PS out, RConn(2)=~ out
%   Wye-Connected Load              : LConn(1)=~ (3-phase), RConn(1)=N (1-phase)
%   RLC (3ph, composite)            : LConn(1)/RConn(1) both 3-phase
%   Grounded Neutral (3ph)          : LConn(1)=3-phase ground
%   Electrical Reference            : LConn(1)=1-phase ground
%   Simulink-PS Converter           : Inport(1)=Simulink in, RConn(1)=PS out
%   PS-Simulink Converter           : LConn(1)=PS in, Outport(1)=Simulink out

% --- Parameters from base workspace (Rule 8: never hardcode case5) ---
mg = evalin('base', 'mg');
f0 = evalin('base', 'f0');

% --- Subsystem container ---
ssName = 'TransmissionNetwork';
ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 200 360 320]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% --- Bus layout (pixels inside the subsystem) ---
busXY = struct( ...
    'b1', [ 400  300], ...
    'b2', [ 400  760], ...   % PCC
    'b3', [ 950  760], ...   % 300 MW load
    'b4', [ 950  300], ...   % slack
    'b5', [1500  300]);

% =====================================================================
%   BUS 2 — PCC connection port (exposed to model top level)
% =====================================================================
pccPortPath = [ssPath '/Elec_PCC_230kV'];
add_block('built-in/PMIOPort', pccPortPath, ...
    'Side', 'Left', ...
    'Position', [busXY.b2(1)-200, busXY.b2(2)-10, busXY.b2(1)-180, busXY.b2(2)+10]);

% =====================================================================
%   BUS 4 — slack: ideal Voltage Source (Three-Phase), 230 kV, f0, 0deg
% =====================================================================
slackPath = [ssPath '/Slack_Bus4'];
add_block('ee_lib/Sources/Voltage Source (Three-Phase)', slackPath, ...
    'Position', [busXY.b4(1)-40, busXY.b4(2)-110, busXY.b4(1)+40, busXY.b4(2)-30]);
set_param(slackPath, ...
    'vline_rms',        '230e3', ...
    'freq',             num2str(f0), ...
    'shift',            '0', ...
    'impedance_option', '0');         % ideal/stiff source — NOT '1' (default
                                      % '1' gives a huge internal impedance that
                                      % collapses the bus under load)
% Slack neutral (1-phase) -> single-phase Electrical Reference
slackGnd = [ssPath '/Slack_Bus4_N'];
add_block('ee_lib/Connectors & References/Electrical Reference', slackGnd, ...
    'Position', [busXY.b4(1)-110, busXY.b4(2)-100, busXY.b4(1)-80, busXY.b4(2)-70]);
add_line(ssPath, conn(slackPath,'LConn',1), conn(slackGnd,'LConn',1));

% =====================================================================
%   BUS 1, 3, 5 — Classical SM subsystems (aggregate gen per bus)
% =====================================================================
agg = aggregate_gen(mg.gen);
sm1 = add_classical_sm(ssPath, 'SM_Bus1', 1, agg, mg, f0, busXY.b1);
sm3 = add_classical_sm(ssPath, 'SM_Bus3', 3, agg, mg, f0, busXY.b3);
sm5 = add_classical_sm(ssPath, 'SM_Bus5', 5, agg, mg, f0, busXY.b5);

% =====================================================================
%   LOADS — Bus 3 (300 MW) and Bus 4 (400 MW)
% =====================================================================
load3 = add_wye_load(ssPath, 'Load_Bus3', 3, mg, f0, ...
    [busXY.b3(1)+120, busXY.b3(2)+60, busXY.b3(1)+180, busXY.b3(2)+140]);
load4 = add_wye_load(ssPath, 'Load_Bus4', 4, mg, f0, ...
    [busXY.b4(1)+120, busXY.b4(2)+60, busXY.b4(1)+180, busXY.b4(2)+140]);

% =====================================================================
%   BUS RAILS — one representative conserving port per bus node
% =====================================================================
rail = struct();
rail.b1 = boundary_conn(sm1);                 % SM terminal (3-phase)
rail.b2 = conn(pccPortPath,'RConn',1);        % PMIOPort inside port
rail.b3 = boundary_conn(sm3);
rail.b4 = conn(slackPath,'RConn',1);          % slack ~ (3-phase output)
rail.b5 = boundary_conn(sm5);

% =====================================================================
%   BRANCHES — 6 PI sections (Series RL + C/2 shunt at each end)
% =====================================================================
nbr = numel(mg.branch.from);
for k = 1:nbr
    f_id = mg.branch.from(k);
    t_id = mg.branch.to(k);
    posF = busXY.(sprintf('b%d', f_id));
    posT = busXY.(sprintf('b%d', t_id));
    midX = round((posF(1)+posT(1))/2);
    midY = round((posF(2)+posT(2))/2);
    [pF, pT] = add_pi_branch(ssPath, sprintf('Br_%d_%d', f_id, t_id), ...
        mg.branch.R_ohm(k), mg.branch.L_henry(k), mg.branch.C_farad(k), ...
        [midX-100, midY-30, midX+100, midY+30]);
    add_line(ssPath, rail.(sprintf('b%d', f_id)), pF);
    add_line(ssPath, rail.(sprintf('b%d', t_id)), pT);
end

% =====================================================================
%   LOAD CONNECTIONS — phase to bus rail (neutral already grounded inside)
% =====================================================================
add_line(ssPath, rail.b3, conn(load3,'LConn',1));
add_line(ssPath, rail.b4, conn(load4,'LConn',1));

fprintf('  build_transmission: 5 buses, %d branches, 3 SMs, 2 loads.\n', nbr);
end % build_transmission


% =====================================================================
%                              HELPERS
% =====================================================================

function agg = aggregate_gen(gen)
% Aggregate multi-generator buses into a per-bus struct (Pg sum, Vg mean).
buses = unique(gen.bus);
agg = struct();
for k = 1:numel(buses)
    bid  = buses(k);
    idx  = gen.bus == bid;
    fld  = sprintf('b%d', bid);
    agg.(fld).Pg_MW    = sum(gen.Pg_MW(idx));
    agg.(fld).Qg_MVAr  = sum(gen.Qg_MVAr(idx));
    agg.(fld).Vg_pu    = mean(gen.Vg_pu(idx));
    agg.(fld).Sbase_MW = max(sum(gen.Pmax_MW(idx)), agg.(fld).Pg_MW*1.2);
end
end


function smPath = add_classical_sm(parent, name, busId, agg, mg, f0, busPos)
% Classical 2nd-order synchronous machine (IO_CONTRACTS §3.2):
%   swing_eq -> omega integrator -> theta integrator -> 3-phase cos
%   generator -> Simulink-PS -> Controlled Voltage Source (instantaneous)
%   behind transient reactance X' -> current/voltage sensors -> terminal.

smPath = [parent '/' name];
add_block('built-in/Subsystem', smPath, ...
    'Position', [busPos(1)-60, busPos(2)-60, busPos(1)+60, busPos(2)+60]);
Simulink.SubSystem.deleteContents(smPath);

fld       = sprintf('b%d', busId);
P_mech_MW = agg.(fld).Pg_MW;
Qg_MVAr   = agg.(fld).Qg_MVAr;
Sbase_MW  = agg.(fld).Sbase_MW;

H_sec = 5.5;     % inertia constant [s]  (PROJECT_KNOWLEDGE §5)
D_pu  = 1.5;     % damping [pu/pu]
Xp_pu = 0.25;    % transient reactance on machine base

busIdx = find(mg.bus.id == busId);

% Internal EMF behind X' at the runpf operating point (classical-model
% initialisation). Holding |E| and angle(E) fixed makes the machine settle
% at the runpf terminal voltage despite having no AVR. All phasors are in
% per-unit on the 230 kV / machine-MVA base.
Vt_pu = mg.bus.Vm_pu(busIdx) * exp(1i*mg.bus.Va_rad(busIdx));   % terminal V
S_pu  = (P_mech_MW + 1i*Qg_MVAr) / Sbase_MW;                    % gen output
I_pu  = conj(S_pu / Vt_pu);                                     % terminal current
E_pu  = Vt_pu + 1i*Xp_pu*I_pu;                                  % EMF behind X'
delta_init = angle(E_pu);                                       % rotor angle [rad]

V_base_LL_RMS = 230e3;
V_phase_peak  = V_base_LL_RMS * sqrt(2/3);           % ~187.79 kV
V_mag_peak    = V_phase_peak * abs(E_pu);            % internal EMF peak [V]

Zbase_machine = V_base_LL_RMS^2 / (Sbase_MW*1e6);    % machine base [ohm]
Xp_ohm        = Xp_pu * Zbase_machine;
Lp_henry      = Xp_ohm / (2*pi*f0);

% ----- Control-side constants -----
add_block('built-in/Constant', [smPath '/Pmech_MW'], 'Value', num2str(P_mech_MW), ...
    'Position', [40 40 100 70]);
add_block('built-in/Constant', [smPath '/H_sec'],    'Value', num2str(H_sec), ...
    'Position', [40 100 100 130]);
add_block('built-in/Constant', [smPath '/D_pu'],     'Value', num2str(D_pu), ...
    'Position', [40 150 100 180]);
add_block('built-in/Constant', [smPath '/Sbase_MW'], 'Value', num2str(Sbase_MW), ...
    'Position', [40 200 100 230]);

% ----- swing equation (Rule 2 form: tf([1],[2H, D])) -----
swingBlk = [smPath '/Swing_Eq'];
add_block('simulink/User-Defined Functions/MATLAB Function', swingBlk, ...
    'Position', [200 90 320 190]);
set_mlfcn(swingBlk, { ...
 'function domega_pu_dt = swing_eq(P_mech_MW, P_elec_MW, omega_pu, H_sec, D_pu, S_base_MW)'
 '%#codegen'
 '% Classical swing equation, per-unit, Rule-2 form 1/(2H*s + D).'
 '% Per-unit base = machine MVA rating (Rule 3, single-machine row).'
 '    dP_pu        = (P_mech_MW - P_elec_MW) / S_base_MW;'
 '    dOmega_pu    = omega_pu - 1.0;'
 '    domega_pu_dt = (dP_pu - D_pu * dOmega_pu) / (2 * H_sec);'
 'end'});

% omega integrator (IC = 1.0 pu)
add_block('simulink/Continuous/Integrator', [smPath '/Int_omega'], ...
    'InitialCondition', '1.0', 'Position', [360 120 400 160]);
% absolute angular velocity = 2*pi*f0 * omega_pu
add_block('simulink/Math Operations/Gain', [smPath '/G_2pif0'], ...
    'Gain', num2str(2*pi*f0), 'Position', [430 120 480 160]);
% theta integrator (IC = delta_init rad)
add_block('simulink/Continuous/Integrator', [smPath '/Int_theta'], ...
    'InitialCondition', num2str(delta_init), 'Position', [510 120 550 160]);

% 3-phase phase offsets
add_block('simulink/Sources/Constant', [smPath '/Off_b'], 'Value', num2str(-2*pi/3), ...
    'Position', [510 230 560 260]);
add_block('simulink/Sources/Constant', [smPath '/Off_c'], 'Value', num2str(+2*pi/3), ...
    'Position', [510 280 560 310]);
add_block('simulink/Math Operations/Sum', [smPath '/Sum_a'], 'Inputs', '+',  'Position', [590 125 615 150]);
add_block('simulink/Math Operations/Sum', [smPath '/Sum_b'], 'Inputs', '++', 'Position', [590 180 615 205]);
add_block('simulink/Math Operations/Sum', [smPath '/Sum_c'], 'Inputs', '++', 'Position', [590 235 615 260]);
add_block('simulink/Math Operations/Trigonometric Function', [smPath '/Cos_a'], 'Operator','cos','Position',[640 122 670 152]);
add_block('simulink/Math Operations/Trigonometric Function', [smPath '/Cos_b'], 'Operator','cos','Position',[640 177 670 207]);
add_block('simulink/Math Operations/Trigonometric Function', [smPath '/Cos_c'], 'Operator','cos','Position',[640 232 670 262]);
add_block('simulink/Math Operations/Gain', [smPath '/Va'], 'Gain', num2str(V_mag_peak), 'Position', [700 122 745 152]);
add_block('simulink/Math Operations/Gain', [smPath '/Vb'], 'Gain', num2str(V_mag_peak), 'Position', [700 177 745 207]);
add_block('simulink/Math Operations/Gain', [smPath '/Vc'], 'Gain', num2str(V_mag_peak), 'Position', [700 232 745 262]);
add_block('simulink/Signal Routing/Mux', [smPath '/Vabc'], 'Inputs', '3', 'Position', [770 122 780 262]);

% Simulink -> PS (volts) feeding the controlled voltage source.
% Input filtering is REQUIRED: prevents an algebraic direct path through the
% Simulink→PS→CVS network under ode14x (Rule 1). tau=1e-4 s << electrical
% period (20 ms) so 50 Hz attenuation is negligible; any residual phase lag
% is absorbed by the rotor-angle integrator at steady state.
add_block('nesl_utility/Simulink-PS Converter', [smPath '/V_S2PS'], ...
    'Position', [810 180 850 210]);
set_param([smPath '/V_S2PS'], 'Unit', 'V', ...
    'FilteringAndDerivatives', 'filter', ...
    'SimscapeFilterOrder', '1', ...
    'InputFilterTimeConstant', '1e-4');

% Controlled Voltage Source (Three-Phase), instantaneous input
cvs = [smPath '/CVS'];
add_block('ee_lib/Sources/Controlled Voltage Source (Three-Phase)', cvs, ...
    'Position', [900 160 980 250]);
set_param(cvs, 'input_option', 'ee.enum.inputType.instantaneous');
% CVS neutral (1-phase) -> Electrical Reference
add_block('ee_lib/Connectors & References/Electrical Reference', [smPath '/CVS_N'], ...
    'Position', [900 290 940 320]);
add_line(smPath, conn(cvs,'LConn',2), conn([smPath '/CVS_N'],'LConn',1));

% Transient reactance X' (Series RL, R tiny)
xp = [smPath '/Xp'];
add_block('ee_lib/Passive/RLC Assemblies/RLC (Three-Phase)', xp, ...
    'Position', [1030 170 1090 240]);
set_param(xp, 'component_structure', 'ee.enum.rlc.structure.SeriesRL', ...
    'R', '0.001', 'L', num2str(Lp_henry));

% Current sensor (series) and voltage sensor (parallel) at the terminal
isens = [smPath '/Isens'];
add_block('ee_lib/Sensors & Transducers/Current Sensor (Three-Phase)', isens, ...
    'Position', [1140 170 1200 240]);
vsens = [smPath '/Vsens'];
add_block('ee_lib/Sensors & Transducers/Phase Voltage Sensor (Three-Phase)', vsens, ...
    'Position', [1260 290 1320 360]);
add_block('ee_lib/Connectors & References/Grounded Neutral (Three-Phase)', [smPath '/Vsens_ref'], ...
    'Position', [1260 400 1320 430]);

% Terminal connection port
term = [smPath '/Term'];
add_block('built-in/PMIOPort', term, 'Side', 'Right', 'Position', [1380 195 1400 215]);

% PS-Simulink converters for sensor outputs (inherit V / A units)
add_block('nesl_utility/PS-Simulink Converter', [smPath '/I_PS2S'], 'Position', [1180 110 1220 140]);
add_block('nesl_utility/PS-Simulink Converter', [smPath '/V_PS2S'], 'Position', [1340 300 1380 330]);

% Pe = sum(v.*i) instantaneous three-phase power
peBlk = [smPath '/Pe_calc'];
add_block('simulink/User-Defined Functions/MATLAB Function', peBlk, 'Position', [1080 60 1180 110]);
set_mlfcn(peBlk, { ...
 'function P_elec_MW = Pe_calc(V_abc, I_abc)'
 '%#codegen'
 '% Instantaneous 3-phase electrical power at the machine terminal [MW].'
 '    P_W       = V_abc(1)*I_abc(1) + V_abc(2)*I_abc(2) + V_abc(3)*I_abc(3);'
 '    P_elec_MW = P_W / 1e6;'
 'end'});

% Pe low-pass (one electrical cycle, 20 ms at 50 Hz)
add_block('simulink/Continuous/Transfer Fcn', [smPath '/Pe_LPF'], ...
    'Numerator', '[1]', 'Denominator', '[0.02 1]', 'Position', [960 60 1030 100]);

% ----- ELECTRICAL wiring -----
add_line(smPath, conn([smPath '/V_S2PS'],'RConn',1), conn(cvs,'LConn',1));  % PS ref -> CVS
add_line(smPath, conn(cvs,'RConn',1),   conn(xp,'LConn',1));                % CVS ~ -> X'
add_line(smPath, conn(xp,'RConn',1),    conn(isens,'LConn',1));             % X' -> Isens in
add_line(smPath, conn(isens,'RConn',2), conn(term,'RConn',1));             % Isens out -> Term
add_line(smPath, conn(isens,'RConn',2), conn(vsens,'LConn',1));            % terminal -> Vsens node
add_line(smPath, conn(vsens,'RConn',2), conn([smPath '/Vsens_ref'],'LConn',1)); % Vsens ref -> gnd

% ----- SENSOR PS -> Simulink -----
add_line(smPath, conn(isens,'RConn',1), conn([smPath '/I_PS2S'],'LConn',1));
add_line(smPath, conn(vsens,'RConn',1), conn([smPath '/V_PS2S'],'LConn',1));

% ----- CONTROL (Simulink) wiring -----
add_line(smPath, 'V_PS2S/1', 'Pe_calc/1', 'autorouting','on');
add_line(smPath, 'I_PS2S/1', 'Pe_calc/2', 'autorouting','on');
add_line(smPath, 'Pe_calc/1', 'Pe_LPF/1', 'autorouting','on');

add_line(smPath, 'Pmech_MW/1', 'Swing_Eq/1', 'autorouting','on');
add_line(smPath, 'Pe_LPF/1',   'Swing_Eq/2', 'autorouting','on');
add_line(smPath, 'Int_omega/1','Swing_Eq/3', 'autorouting','on');
add_line(smPath, 'H_sec/1',    'Swing_Eq/4', 'autorouting','on');
add_line(smPath, 'D_pu/1',     'Swing_Eq/5', 'autorouting','on');
add_line(smPath, 'Sbase_MW/1', 'Swing_Eq/6', 'autorouting','on');

add_line(smPath, 'Swing_Eq/1', 'Int_omega/1','autorouting','on');
add_line(smPath, 'Int_omega/1','G_2pif0/1',  'autorouting','on');
add_line(smPath, 'G_2pif0/1',  'Int_theta/1','autorouting','on');
add_line(smPath, 'Int_theta/1','Sum_a/1','autorouting','on');
add_line(smPath, 'Int_theta/1','Sum_b/1','autorouting','on');
add_line(smPath, 'Int_theta/1','Sum_c/1','autorouting','on');
add_line(smPath, 'Off_b/1','Sum_b/2','autorouting','on');
add_line(smPath, 'Off_c/1','Sum_c/2','autorouting','on');
add_line(smPath, 'Sum_a/1','Cos_a/1','autorouting','on');
add_line(smPath, 'Sum_b/1','Cos_b/1','autorouting','on');
add_line(smPath, 'Sum_c/1','Cos_c/1','autorouting','on');
add_line(smPath, 'Cos_a/1','Va/1','autorouting','on');
add_line(smPath, 'Cos_b/1','Vb/1','autorouting','on');
add_line(smPath, 'Cos_c/1','Vc/1','autorouting','on');
add_line(smPath, 'Va/1','Vabc/1','autorouting','on');
add_line(smPath, 'Vb/1','Vabc/2','autorouting','on');
add_line(smPath, 'Vc/1','Vabc/3','autorouting','on');
add_line(smPath, 'Vabc/1','V_S2PS/1','autorouting','on');
end


function loadPath = add_wye_load(parent, name, busId, mg, f0, posBlock)
% Wye-Connected Load (rated-power P+Q). LConn(1)=3-phase to bus rail,
% RConn(1)=neutral (1-phase) -> Electrical Reference.
loadPath = [parent '/' name];
add_block('ee_lib/Passive/RLC Assemblies/Wye-Connected Load', loadPath, 'Position', posBlock);

busIdx = find(mg.bus.id == busId);
P_W    = mg.bus.Pd_MW(busIdx)   * 1e6;
Q_VAR  = mg.bus.Qd_MVAr(busIdx) * 1e6;
set_param(loadPath, ...
    'parameterization', 'ee.enum.rlc.parameterization.ratedpower', ...
    'VRated', '230e3', ...
    'FRated', num2str(f0), ...
    'P',      num2str(P_W), ...
    'Qpos',   num2str(Q_VAR));

gndPath = [loadPath '_N'];
add_block('ee_lib/Connectors & References/Electrical Reference', gndPath, ...
    'Position', [posBlock(1)+10, posBlock(4)+30, posBlock(1)+50, posBlock(4)+60]);
add_line(parent, conn(loadPath,'RConn',1), conn(gndPath,'LConn',1));
end


function [pFrom, pTo] = add_pi_branch(parent, baseName, R_oh, L_h, C_f, posBox)
% One PI section = Series RL + C/2 shunt at each end (each shunt grounded
% via Grounded Neutral (Three-Phase)). Returns the from/to 3-phase ports
% for the caller to wire to the bus rails.
x1 = posBox(1); yMid = round((posBox(2)+posBox(4))/2);

serPath = [parent '/' baseName '_RL'];
add_block('ee_lib/Passive/RLC Assemblies/RLC (Three-Phase)', serPath, ...
    'Position', [x1+70, yMid-30, x1+130, yMid+30]);
set_param(serPath, 'component_structure', 'ee.enum.rlc.structure.SeriesRL', ...
    'R', num2str(R_oh), 'L', num2str(L_h));

% R_parasitic = 1 ohm series on each shunt C: at 50 Hz Z_C = 1/(wC) ~ 160 kohm,
% so 1 ohm is a ~6 ppm perturbation — electrically invisible, but it breaks the
% index-2 structure of a pure C directly across the ideal slack source. Without
% it the IC solve is structurally singular and converges only on a lucky pivot
% (failed reproducibly once the ESS module added states, 2026-06-10).
sh1 = [parent '/' baseName '_C1'];
add_block('ee_lib/Passive/RLC Assemblies/RLC (Three-Phase)', sh1, ...
    'Position', [x1+10, yMid+45, x1+50, yMid+95]);
set_param(sh1, 'component_structure', 'ee.enum.rlc.structure.C', ...
    'C', num2str(C_f/2), 'R_parasitic', '1');

sh2 = [parent '/' baseName '_C2'];
add_block('ee_lib/Passive/RLC Assemblies/RLC (Three-Phase)', sh2, ...
    'Position', [x1+150, yMid+45, x1+190, yMid+95]);
set_param(sh2, 'component_structure', 'ee.enum.rlc.structure.C', ...
    'C', num2str(C_f/2), 'R_parasitic', '1');

g1 = [parent '/' baseName '_C1_G'];
g2 = [parent '/' baseName '_C2_G'];
add_block('ee_lib/Connectors & References/Grounded Neutral (Three-Phase)', g1, ...
    'Position', [x1+10, yMid+110, x1+50, yMid+140]);
add_block('ee_lib/Connectors & References/Grounded Neutral (Three-Phase)', g2, ...
    'Position', [x1+150, yMid+110, x1+190, yMid+140]);

add_line(parent, conn(sh1,'RConn',1), conn(g1,'LConn',1));
add_line(parent, conn(sh2,'RConn',1), conn(g2,'LConn',1));

% Tie each shunt to the corresponding end of the series branch
add_line(parent, conn(sh1,'LConn',1), conn(serPath,'LConn',1));
add_line(parent, conn(sh2,'LConn',1), conn(serPath,'RConn',1));

pFrom = conn(sh1,'LConn',1);
pTo   = conn(sh2,'LConn',1);
end


function h = conn(blkPath, side, idx)
% Return a conserving/PS port handle by side ('LConn'/'RConn') and index.
ph = get_param(blkPath, 'PortHandles');
h  = ph.(side)(idx);
end


function h = boundary_conn(subPath)
% The single boundary conserving port of a subsystem (Term PMIOPort image).
ph   = get_param(subPath, 'PortHandles');
allc = [ph.LConn(:); ph.RConn(:)];
h    = allc(1);
end


function set_mlfcn(blkPath, lines)
% Set a MATLAB Function block body via the modern configuration API.
% NOTE: do NOT route text through sprintf — '%' chars (e.g. %#codegen,
% comments) would be interpreted as format specifiers and mangled.
cfg = get_param(blkPath, 'MATLABFunctionConfiguration');
cfg.FunctionScript = strjoin(lines, newline);
end
