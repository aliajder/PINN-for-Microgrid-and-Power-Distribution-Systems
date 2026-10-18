function build_dcbus(mdl)
%% build_dcbus.m
% Builds the 1500 V common DC bus subsystem inside the master model.
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.6 (DCBus)
% Master    : SIMULINK/build_microgrid_eelib.m calls this with the model name
%
% WHAT THIS MODULE IS (per IO_CONTRACTS §3.6):
%   A PASSIVE electrical node — the single common DC bus (A1) all
%   inverter-interfaced resources share. Voltage regulation is NOT done
%   here: the ESS bidirectional DC-DC owns V_dc (§3.9); the VSC has a
%   saturation backstop only (§3.5).
%
%      Elec_VSC_term ──●────●────●────●────●────●── (one node, + rail)
%                      │    │    │    │    │    │
%                   VSC   PV  Wind  ESS  Elec  FC      (5 of 6 open
%                      │                                until Tasks 5.6+)
%                 [C_dc 30 mF]──[V sensor ∥]
%                      │             │
%                     gnd       Out_V_dc_V / Out_V_dc_pu
%
% ── DESIGN DECISIONS taken while building (2026-06-10) ────────────────────
%
%  F1. GROUNDED-NEGATIVE RAIL convention. IO_CONTRACTS §3.6 calls the DC
%      ports "2-wire", but a Simscape conserving port is a single node. We
%      implement the bus as: + rail through the boundary ports, − rail =
%      Electrical Reference (ground) inside EVERY DC subsystem. Electrically
%      identical to a 2-wire bus with grounded negative rail (and realistic —
%      utility-scale 1500 V DC systems are functionally grounded). The VSC's
%      internal LV/MV transformer galvanically isolates this DC ground from
%      the 33 kV AC grounding (A9 zig-zag remains THE island AC ground).
%
%  F2. CAPACITOR INITIAL CONDITION vc = 1500 V (priority High). Without it
%      the bus starts at 0 V and the test's stiff DC source dumps a huge
%      inrush through its milli-ohm series R at t=0 (numerically survivable
%      under ode14x but physically meaningless). Verified param interface:
%      ee_lib/Passive/Capacitor uses c [F], r [Ω], vc_specify/vc/vc_priority.
%
%  F3. R2 (CPL instability) note: the tightly-regulated converters that will
%      hang on this bus (VSC, electrolyzer, FC) behave as constant-power
%      loads with NEGATIVE incremental impedance. Mitigation per CLAUDE.md
%      R2 = sufficient C_dc (30 mF per the §3.6 sizing rule: ±50 MW step,
%      τ_pi = 50 ms, ΔV ≤ 5%) + active damping LATER if island tests
%      oscillate. C_dc is therefore a % TUNED placeholder, not a constant.

% ── Parameters (no magic numbers; units in names) ─────────────────────────
V_dc_nom_V = 1500;     % DC bus nominal voltage          [V]  (A1)
C_dc_F     = 30e-3;    % bus capacitance                 [F]  % TUNED: §3.6 sizing, revisit in island test (R2)
R_esr_ohm  = 1e-3;     % capacitor ESR (mild damping)    [Ω]  % TUNED: placeholder, helps R2

% ── Subsystem container ───────────────────────────────────────────────────
ssName = 'DCBus';
ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 1020 360 1160]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY PORTS — six DC terminals (one per resource) + measurements
%   All six tie to the SAME internal node; 5 stay open at top level
%   until the corresponding source modules are built (Tasks 5.6+).
% =====================================================================
termNames = {'Elec_VSC_term','Elec_PV_term','Elec_Wind_term', ...
             'Elec_ESS_term','Elec_Elec_term','Elec_FC_term'};
for k = 1:numel(termNames)
    add_block('built-in/PMIOPort', [ssPath '/' termNames{k}], ...
        'Side', 'Left', 'Position', [40 60+90*(k-1) 60 80+90*(k-1)]);
end
add_block('built-in/Outport', [ssPath '/Out_V_dc_V'],  'Position', [800 100 820 118]);
add_block('built-in/Outport', [ssPath '/Out_V_dc_pu'], 'Position', [800 160 820 178]);

% =====================================================================
%   THE BUS NODE — star-join all terminals on the capacitor's + terminal.
%   Capacitor: verified ee_lib/Passive/Capacitor, LConn(1)/RConn(1) = the
%   two terminals; c=C_dc, small ESR r for damping; IC vc=1500 V (F2).
% =====================================================================
cap = [ssPath '/C_dc'];
add_block('ee_lib/Passive/Capacitor', cap, 'Position', [400 300 460 360]);
set_param(cap, 'c', num2str(C_dc_F), 'r', num2str(R_esr_ohm), ...
    'vc_specify', 'on', 'vc', num2str(V_dc_nom_V), 'vc_priority', 'High');

add_block('ee_lib/Connectors & References/Electrical Reference', [ssPath '/Gnd'], ...
    'Position', [400 420 440 450]);
add_line(ssPath, conn(cap,'RConn',1), conn([ssPath '/Gnd'],'LConn',1));  % − rail = gnd (F1)

busNode = conn(cap,'LConn',1);    % + rail node; everything stars off here
for k = 1:numel(termNames)
    add_line(ssPath, conn([ssPath '/' termNames{k}],'RConn',1), busNode);
end

% =====================================================================
%   VOLTAGE SENSOR (parallel, + rail → gnd) → Out_V_dc_V / Out_V_dc_pu
%   Verified port map: LConn(1)=+, RConn(1)=PS out, RConn(2)=−.
% =====================================================================
vs = [ssPath '/V_dc_sens'];
add_block('ee_lib/Sensors & Transducers/Voltage Sensor', vs, 'Position', [560 280 620 340]);
add_line(ssPath, busNode, conn(vs,'LConn',1));
add_line(ssPath, conn(vs,'RConn',2), conn([ssPath '/Gnd'],'LConn',1));

add_block('nesl_utility/PS-Simulink Converter', [ssPath '/Vdc_PS2S'], 'Position', [680 95 720 125]);
set_param([ssPath '/Vdc_PS2S'], 'Unit', 'V');
add_line(ssPath, conn(vs,'RConn',1), conn([ssPath '/Vdc_PS2S'],'LConn',1));
add_line(ssPath, 'Vdc_PS2S/1', 'Out_V_dc_V/1', 'autorouting','on');

% pu companion output (base 1500 V, §1 unit table)
add_block('simulink/Math Operations/Gain', [ssPath '/V2pu'], ...
    'Gain', num2str(1/V_dc_nom_V), 'Position', [680 155 720 185]);
add_line(ssPath, 'Vdc_PS2S/1', 'V2pu/1', 'autorouting','on');
add_line(ssPath, 'V2pu/1', 'Out_V_dc_pu/1', 'autorouting','on');

fprintf('  build_dcbus: 1500 V DC node, C_dc=%.0f mF (IC 1500 V), 6 terminals, V sensor.\n', C_dc_F*1e3);
end % build_dcbus


% =====================================================================
%                              HELPERS
% =====================================================================
function h = conn(blkPath, side, idx)
% Conserving/PS port handle by side ('LConn'/'RConn') and index.
ph = get_param(blkPath, 'PortHandles');
h  = ph.(side)(idx);
end
