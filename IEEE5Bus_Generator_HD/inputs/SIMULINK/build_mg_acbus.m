function build_mg_acbus(mdl)
%% build_mg_acbus.m
% Builds the 33 kV Microgrid AC bus + the permanent zig-zag grounding
% transformer inside the master model.
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.4 (MicrogridACBus) + §3.4.1 (ZigZagGrounding)
% Master    : SIMULINK/build_microgrid_eelib.m calls this with the model name
% Params    : f0 read from base workspace (Rule 10)
%
% WHAT THIS MODULE IS (per IO_CONTRACTS §3.4 / §3.4.1):
%
%   PCC/Elec_MV_33kV ──~33kV~──►│ Elec_PCC_in
%                               │   [I_bus sensor]──● 33 kV bus node ●── Elec_VSC_in (open until Task 5.5)
%                               │                   │            │
%                               │     [Wye Variable Load (lagging)]   [ZigZagGrounding] (A9, permanent)
%                               │       300 MW + Q@pf0.95             zig-zag earthing XFMR + NGR
%                               │            │ (neutral→gnd)               │ neutral→I sensor→NGR→gnd
%
% ── ZIG-ZAG GROUNDING DISCOVERY (2026-06-08, before build) ────────────────
%   ee_lib SHIPS a native zig-zag grounding transformer:
%     ee_lib/Passive/Transformers/Earthing Transformer
%   (high +/−-sequence impedance, low zero-sequence path = textbook zig-zag
%   grounding bank). A heavier `Zigzag-Delta-Wye Transformer` (3-winding) also
%   exists, but the Earthing Transformer is the dedicated grounding device →
%   we use it. No zero-sequence-shunt equivalent needed.
%   Verified port map (probed 2026-06-08):
%     Earthing Transformer : LConn(1)=3-phase ~ terminal, RConn(1)=neutral n
%     Current Sensor (1-φ) : LConn(1)=elec in, RConn(2)=elec out, RConn(1)=PS signal
%
% ── DESIGN DECISIONS taken while building (2026-06-08) ────────────────────
%
%  E1. LOAD BLOCK = "Wye-Connected Variable Load (lagging)" (wye_series_RL),
%      NOT the plain "Wye-Connected Variable Load" — the plain one is
%      RESISTIVE-ONLY (wye_R, P with no Q). The lagging variant exposes two
%      physical-signal inputs (LConn(2)=P [W], LConn(3)=Q [VAr]) so we can
%      drive both active and reactive demand — ready for the ENTSO-E real
%      profile later. P/Q arrive on the subsystem boundary in MW/MVAr and are
%      scaled ×1e6 → W/VAr at the Simulink-PS converters (IO_CONTRACTS §1.2).
%
%  E2. ZigZagGrounding is built as a NESTED subsystem of MicrogridACBus
%      (MicrogridACBus/ZigZagGrounding), tapped in PARALLEL on the internal
%      33 kV bus node (NOT in the PCC↔VSC series path). This keeps the ground
%      reference PERMANENTLY bonded to the bus and independent of CB1 — exactly
%      A9's intent (island mode keeps its ground when CB1 opens). It still
%      satisfies §3.4.1 (a named subsystem with an Elec_AC_33kV port).
%
%  E3. NGR (neutral grounding resistor) is included between the earthing-XFMR
%      neutral and Electrical Reference, with a 1-φ current sensor in series →
%      Out_I_neutral_A. R_ngr is a `% TUNED:` placeholder (resistance-grounded
%      design; size it in the Task-5.4 fault/island test). Set R_ngr→~0 for
%      effectively-solid grounding. In BALANCED operation no zero-sequence
%      current flows, so the neutral current ≈ 0 regardless of R_ngr — that is
%      the quantity the standalone test checks.

% ── Parameters (no magic numbers; units in names) ─────────────────────────
f0_Hz       = evalin('base','f0');     % 50 Hz (Rule 10)

V_bus_LL    = 33e3;     % MG bus line-line RMS               [V]   (A3)
% Load pf is 0.95 lagging (documented); Q is derived upstream and arrives on
% In_Q_load_MVAr — the bus itself stays a pure passive element (§3.4 / §1.2).

% Zig-zag (earthing) grounding transformer — A9. Grounding banks are
% short-time rated; a modest continuous-equivalent rating keeps the
% magnetizing (normal) current negligible while giving a low zero-seq path.
S_zz_VA     = 2e6;      % grounding-XFMR rating              [VA]  % TUNED: placeholder
Xl_zz_pu    = 0.30;     % leakage (≈ zero-sequence series Z) [pu]  (block default)
Rw_zz_pu    = 0.01;     % winding resistance                [pu]  (block default)
R_ngr_ohm   = 47;       % neutral grounding resistor        [Ω]   % TUNED: placeholder (≈400 A SLG limit @33 kV; size in fault test)

% ── Subsystem container ───────────────────────────────────────────────────
ssName = 'MicrogridACBus';
ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 620 360 760]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY PORTS
% =====================================================================
% Electrical: PCC feed on the left, VSC tie on the right (open until Task 5.5).
pccPort = [ssPath '/Elec_PCC_in'];
add_block('built-in/PMIOPort', pccPort, 'Side', 'Left',  'Position', [40 300 60 320]);
vscPort = [ssPath '/Elec_VSC_in'];
add_block('built-in/PMIOPort', vscPort, 'Side', 'Right', 'Position', [1000 300 1020 320]);

% Control inputs (load demand) and measurement outputs.
add_block('built-in/Inport',  [ssPath '/In_P_load_MW'],   'Position', [40 120 60 138]);
add_block('built-in/Inport',  [ssPath '/In_Q_load_MVAr'], 'Position', [40 170 60 188]);
add_block('built-in/Outport', [ssPath '/Out_I_bus_A'],    'Position', [1000 120 1020 138]);
add_block('built-in/Outport', [ssPath '/Out_I_neutral_A'],'Position', [1000 600 1020 618]);

% =====================================================================
%   BUS CURRENT SENSOR (series on the PCC feed) → Out_I_bus_A
%   Measures the 3-φ current delivered from PCC into the MG bus.
% =====================================================================
isens = [ssPath '/I_bus'];
add_block('ee_lib/Sensors & Transducers/Current Sensor (Three-Phase)', isens, ...
    'Position', [180 280 240 360]);
add_line(ssPath, conn(pccPort,'RConn',1), conn(isens,'LConn',1));   % PCC feed → I in

% bus node = current-sensor electrical OUT (RConn(2)); branches off it below.
busNode = conn(isens,'RConn',2);

% I_bus PS → Simulink → Out_I_bus_A
add_block('nesl_utility/PS-Simulink Converter', [ssPath '/Ibus_PS2S'], 'Position', [300 200 340 230]);
add_line(ssPath, conn(isens,'RConn',1), conn([ssPath '/Ibus_PS2S'],'LConn',1));
add_line(ssPath, 'Ibus_PS2S/1', 'Out_I_bus_A/1', 'autorouting','on');

% =====================================================================
%   WYE-CONNECTED VARIABLE LOAD (lagging)  — P & Q signal-driven (E1)
%   Verified port map (probed 2026-06-08):
%     LConn(1)=P demand [W] (PS), LConn(2)=Q demand [VAr] (PS),
%     LConn(3)=3-φ ~ terminal,    RConn(1)=neutral.
% =====================================================================
vload = [ssPath '/Load_300MW'];
add_block('ee_lib/Passive/RLC Assemblies/Wye-Connected Variable Load (lagging)', vload, ...
    'Position', [560 280 640 380]);
set_param(vload, 'VRated', num2str(V_bus_LL), 'FRated', num2str(f0_Hz));

% Load 3-φ terminal ← bus node
add_line(ssPath, busNode, conn(vload,'LConn',3));
% Load neutral → single-phase Electrical Reference
add_block('ee_lib/Connectors & References/Electrical Reference', [ssPath '/Load_N'], ...
    'Position', [560 420 600 450]);
add_line(ssPath, conn(vload,'RConn',1), conn([ssPath '/Load_N'],'LConn',1));

% --- P demand path: In_P_load_MW (MW) ×1e6 → W → PS → LConn(2) ---
add_block('simulink/Math Operations/Gain', [ssPath '/P_MW2W'], ...
    'Gain', '1e6', 'Position', [120 115 160 145]);
add_block('nesl_utility/Simulink-PS Converter', [ssPath '/P_S2PS'], 'Position', [380 115 420 145]);
set_param([ssPath '/P_S2PS'], 'Unit', 'W');
add_line(ssPath, 'In_P_load_MW/1', 'P_MW2W/1', 'autorouting','on');
add_line(ssPath, 'P_MW2W/1',       'P_S2PS/1', 'autorouting','on');
add_line(ssPath, conn([ssPath '/P_S2PS'],'RConn',1), conn(vload,'LConn',1));

% --- Q demand path: In_Q_load_MVAr (MVAr) ×1e6 → VAr → PS → LConn(3) ---
add_block('simulink/Math Operations/Gain', [ssPath '/Q_MVAr2VAr'], ...
    'Gain', '1e6', 'Position', [120 165 160 195]);
add_block('nesl_utility/Simulink-PS Converter', [ssPath '/Q_S2PS'], 'Position', [380 165 420 195]);
set_param([ssPath '/Q_S2PS'], 'Unit', 'V*A');
add_line(ssPath, 'In_Q_load_MVAr/1', 'Q_MVAr2VAr/1', 'autorouting','on');
add_line(ssPath, 'Q_MVAr2VAr/1',     'Q_S2PS/1',     'autorouting','on');
add_line(ssPath, conn([ssPath '/Q_S2PS'],'RConn',1), conn(vload,'LConn',2));

% =====================================================================
%   VSC TIE — expose the bus at the (right) VSC port; open until Task 5.5.
% =====================================================================
add_line(ssPath, busNode, conn(vscPort,'RConn',1));

% =====================================================================
%   ZIG-ZAG GROUNDING (A9) — nested subsystem, parallel on the bus (E2/E3)
% =====================================================================
zzPort = add_zigzag_grounding(ssPath, 'ZigZagGrounding', ...
    V_bus_LL, f0_Hz, S_zz_VA, Xl_zz_pu, Rw_zz_pu, R_ngr_ohm);
% Tap the zig-zag 3-φ terminal onto the bus node (permanent, parallel).
add_line(ssPath, busNode, zzPort);
% Bubble the nested neutral-current measurement up to the bus outport.
add_line(ssPath, 'ZigZagGrounding/1', 'Out_I_neutral_A/1', 'autorouting','on');

fprintf('  build_mg_acbus: 33 kV bus + Wye Variable Load (lagging) + zig-zag Earthing XFMR (A9, permanent).\n');
end % build_mg_acbus


% =====================================================================
%                              HELPERS
% =====================================================================

function termPort = add_zigzag_grounding(parent, name, V_LL, f0, S_VA, Xl_pu, Rw_pu, R_ngr)
% ZigZagGrounding (IO_CONTRACTS §3.4.1, CLAUDE.md A9).
% Native ee_lib Earthing Transformer (zig-zag grounding bank) + NGR +
% neutral current sensor. Permanently connected; passive (no control inputs).
%
%   Elec_AC_33kV ──► [Earthing XFMR ~]      (LConn1 = 3-φ terminal)
%                          │ neutral (RConn1)
%                    [I_neutral sensor] ──► Out_I_neutral_A
%                          │
%                        [NGR R_ngr]
%                          │
%                   Electrical Reference (ground)
%
% Returns the subsystem's 3-φ boundary port handle for the caller to tap
% onto the bus node.

ssPath = [parent '/' name];
add_block('built-in/Subsystem', ssPath, 'Position', [560 520 700 640]);
Simulink.SubSystem.deleteContents(ssPath);

% Boundary: 3-φ electrical IN (left) + neutral-current measurement OUT.
acPort = [ssPath '/Elec_AC_33kV'];
add_block('built-in/PMIOPort', acPort, 'Side', 'Left', 'Position', [40 200 60 220]);
add_block('built-in/Outport', [ssPath '/Out_I_neutral_A'], 'Position', [560 100 580 118]);

% Earthing (zig-zag) transformer. LConn(1)=3-φ ~, RConn(1)=neutral n.
et = [ssPath '/EarthingXFMR'];
add_block('ee_lib/Passive/Transformers/Earthing Transformer', et, ...
    'Position', [160 180 260 260]);
set_param(et, ...
    'SRated', num2str(S_VA), ...
    'VRated', num2str(V_LL), ...
    'FRated', num2str(f0), ...
    'pu_Xl',  num2str(Xl_pu), ...
    'pu_Rw',  num2str(Rw_pu));
add_line(ssPath, conn(acPort,'RConn',1), conn(et,'LConn',1));   % bus → 3-φ terminal

% Neutral current sensor (1-φ): LConn(1)=in, RConn(2)=out, RConn(1)=PS signal.
ns = [ssPath '/I_neutral'];
add_block('ee_lib/Sensors & Transducers/Current Sensor', ns, ...
    'Position', [340 180 400 240]);
add_line(ssPath, conn(et,'RConn',1), conn(ns,'LConn',1));        % XFMR neutral → sensor in

% NGR (neutral grounding resistor) → Electrical Reference.
ngr = [ssPath '/NGR'];
add_block('ee_lib/Passive/Resistor', ngr, 'Position', [460 200 520 260]);
set_param(ngr, 'R', num2str(R_ngr));
add_block('ee_lib/Connectors & References/Electrical Reference', [ssPath '/Gnd'], ...
    'Position', [460 300 500 330]);
add_line(ssPath, conn(ns,'RConn',2), conn(ngr,'LConn',1));       % sensor out → NGR
add_line(ssPath, conn(ngr,'RConn',1), conn([ssPath '/Gnd'],'LConn',1));

% Neutral current PS → Simulink → Out_I_neutral_A.
add_block('nesl_utility/PS-Simulink Converter', [ssPath '/In_PS2S'], 'Position', [440 100 480 130]);
set_param([ssPath '/In_PS2S'], 'Unit', 'A');
add_line(ssPath, conn(ns,'RConn',1), conn([ssPath '/In_PS2S'],'LConn',1));
add_line(ssPath, 'In_PS2S/1', 'Out_I_neutral_A/1', 'autorouting','on');

% Return the subsystem's 3-φ boundary port (image of acPort at parent level).
ph   = get_param(ssPath, 'PortHandles');
allc = [ph.LConn(:); ph.RConn(:)];
termPort = allc(1);
end


function h = conn(blkPath, side, idx)
% Conserving/PS port handle by side ('LConn'/'RConn') and index.
ph = get_param(blkPath, 'PortHandles');
h  = ph.(side)(idx);
end
