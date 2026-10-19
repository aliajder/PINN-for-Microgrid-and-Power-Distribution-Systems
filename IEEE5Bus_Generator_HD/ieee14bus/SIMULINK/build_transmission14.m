function build_transmission14(mdl)
%% build_transmission14.m
% TransmissionNetwork = MODIFIYE IEEE 14 bara (19 bara, Demetriou 2017).
% Base workspace degiskeni: mg14 (ieee14bus/mg14_params.mat).
%
% Eleman listesi
%   * 5 ClassicalSM (SM_Bus15/16/17/19/18), makine tabani pu parametreleri
%   * 20 hat/trafo dali (hat = PI; 4-7, 4-9, 5-6 = Yg-Yg trafo) + 5 unite trafosu
%   * 11 sabit-Z yuk (wye), mg14.bus.Vload_V'de kalibre (denge tam)
%   * Bara 19 (13.8 kV) PMIOPort 'Elec_PCC_13p8kV' : mikro sebekenin baglanti ucu
% Ideal kaynak YOK: referans frekansi makineler belirler.
% ee_lib port haritasi: inputs/SIMULINK/build_transmission.m basligina bakin.

mg14 = evalin('base', 'mg14');
f0   = mg14.base.fbase;

ssPath = [mdl '/TransmissionNetwork'];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 200 360 320]);
    Simulink.SubSystem.deleteContents(ssPath);
end

nb = numel(mg14.bus.id);
busXY = zeros(nb, 2);
for b = 1:nb
    busXY(b,:) = [400 + 700*mod(b-1,5), 300 + 650*floor((b-1)/5)];
end

rail = cell(nb,1);     % bara dugumu = ilk baglanan elemanin portu

pccPort = [ssPath '/Elec_PCC_13p8kV'];
add_block('built-in/PMIOPort', pccPort, 'Side', 'Left', ...
    'Position', [busXY(19,1)-200, busXY(19,2)-10, busXY(19,1)-180, busXY(19,2)+10]);

% ---- makineler ----
for gi = 1:numel(mg14.gen.unit_bus)
    ub = mg14.gen.unit_bus(gi);
    sm = add_classical_sm(ssPath, sprintf('SM_Bus%d', ub), gi, mg14, f0, busXY(ub,:));
    rail{ub} = boundary_conn(sm);
end

% ---- hatlar / trafolar ----
Sb_W = mg14.base.Sbase_MVA * 1e6;
for k = 1:numel(mg14.branch.from)
    f = mg14.branch.from(k); t = mg14.branch.to(k);
    mid = round((busXY(f,:) + busXY(t,:)) / 2);
    box = [mid(1)-100, mid(2)-30, mid(1)+100, mid(2)+30];
    name = sprintf('Br_%d_%d', f, t);
    if mg14.branch.isXfmr(k)
        if mg14.branch.isUnitTx(k)
            S = mg14.branch.unitTxS_MVA * 1e6;
            xl = mg14.branch.unitTxLeak_pu;  rw = 1e-5;
        else
            S = Sb_W;
            xl = mg14.branch.X_pu(k);        rw = max(mg14.branch.R_pu(k), 1e-5);
        end
        xf = [ssPath '/' name];
        add_block('ee_lib/Passive/Transformers/Two-Winding Transformer (Three-Phase)', xf, 'Position', box);
        set_param(xf, ...
            'SRated', num2str(S), 'FRated', num2str(f0), ...
            'Winding1Connection', 'ee.enum.windingconnection.Yg', ...
            'VRated1', num2str(mg14.branch.V1rated_kV(k)*1e3), ...
            'Winding2Connection', 'ee.enum.windingconnection.Yg', ...
            'VRated2', num2str(mg14.branch.V2rated_kV(k)*1e3), ...
            'leakage_reactance_option', 'ee.enum.transformer_leakage.include', ...
            'pu_Xl1', num2str(xl/2), 'pu_Xl2', num2str(xl/2), ...
            'pu_Rw1', num2str(rw/2), 'pu_Rw2', num2str(rw/2));
        pF = conn(xf,'LConn',1);  pT = conn(xf,'RConn',1);
    else
        [pF, pT] = add_pi_branch(ssPath, name, max(mg14.branch.R_ohm(k),1e-4), ...
            mg14.branch.L_henry(k), mg14.branch.C_farad(k), box);
    end
    rail{f} = attach(ssPath, rail{f}, pF);
    rail{t} = attach(ssPath, rail{t}, pT);
end

% ---- yukler (mikro sebeke yuku haric: o PCC'nin ardinda) ----
nl = 0;
for b = 1:nb
    P = mg14.bus.Pd_MW(b)   - mg14.bus.MGload_MW(b);
    Q = mg14.bus.Qd_MVAr(b) - mg14.bus.MGload_MVAr(b);
    if abs(P) < 1e-9 && abs(Q) < 1e-9, continue; end
    ld = add_wye_load(ssPath, sprintf('Load_Bus%d', b), P, Q, mg14.bus.Vload_V(b), f0, ...
        [busXY(b,1)+120, busXY(b,2)+60, busXY(b,1)+180, busXY(b,2)+140]);
    rail{b} = attach(ssPath, rail{b}, conn(ld,'LConn',1));
    nl = nl + 1;
end

% ---- PCC ucu (bara 19) ----
attach(ssPath, rail{19}, conn(pccPort,'RConn',1));

fprintf('  build_transmission14: %d bara, %d dal, 5 SM, %d yuk.\n', nb, numel(mg14.branch.from), nl);
end


function r = attach(ss, r, port)
% Bara dugumune bir port bagla (ilk port dugumun temsilcisi olur).
if isempty(r)
    r = port;
else
    add_line(ss, r, port);
end
end


% ===== yardimcilar =====

function smPath = add_classical_sm(parent, name, gi, mg14, f0, busPos)
% Classical 2nd-order synchronous machine (IO_CONTRACTS §3.2):
%   swing_eq -> omega integrator -> theta integrator -> 3-phase cos
%   generator -> Simulink-PS -> Controlled Voltage Source (instantaneous)
%   behind transient reactance X' -> current/voltage sensors -> terminal.

smPath = [parent '/' name];
add_block('built-in/Subsystem', smPath, ...
    'Position', [busPos(1)-60, busPos(2)-60, busPos(1)+60, busPos(2)+60]);
Simulink.SubSystem.deleteContents(smPath);

Sbase_MW  = mg14.gen.S_MVA(gi);
P_mech_MW = mg14.gen.Pg_MW(gi);
H_sec     = mg14.gen.H(gi);
D_pu      = mg14.gen.D(gi);
delta_init = mg14.gen.delta0_rad(gi);
V_mag_peak = mg14.gen.Vemf_peak_V(gi);
Lp_henry   = mg14.gen.Lp_henry(gi);
Rs_ohm     = max(mg14.gen.Ra_ohm(gi), 1e-3);

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
    'R', num2str(Rs_ohm), 'L', num2str(Lp_henry));

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


function loadPath = add_wye_load(parent, name, P_MW, Q_MVAr, V_LL, f0, posBlock)
% Sabit-Z yuk: VRated = runpf gerilimi -> P,Q tam o gerilimde cekilir.
loadPath = [parent '/' name];
add_block('ee_lib/Passive/RLC Assemblies/Wye-Connected Load', loadPath, 'Position', posBlock);
set_param(loadPath, ...
    'parameterization', 'ee.enum.rlc.parameterization.ratedpower', ...
    'VRated', num2str(V_LL), 'FRated', num2str(f0), ...
    'P', num2str(P*1e6), 'Qpos', num2str(Q_MVAr*1e6));
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
