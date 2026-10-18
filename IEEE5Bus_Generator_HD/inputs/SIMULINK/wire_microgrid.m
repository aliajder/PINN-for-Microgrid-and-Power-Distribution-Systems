function wire_microgrid(mdl)
%% wire_microgrid.m  — FULL-MODEL inter-module wiring (Task: full-model wiring)
% Connects the 12 already-built module subsystems of microgrid_case5 into one
% running model. Called by build_microgrid_eelib.m AFTER all modules + the H2
% chain are built. Idempotent-ish: every connection is GUARDED (skipped if the
% line already exists or a port is missing) so re-running the master is safe.
%
% Reference : SIMULINK/IO_CONTRACTS.md §4 CONNECTION MAP + §3.x port tables.
% Scope     : permanent TOPOLOGY wiring (electrical + control). The profile
%             SOURCES are CONSTANT placeholders (PV 800 W/m²·25 °C, Wind 9 m/s,
%             load 300 MW @0.95) until PARAMS/load_real_profiles.m exists — they
%             are tagged Src_* so the From-Workspace swap is a one-liner later.
%             VSC f_ref/V_ref/mode and CB_PCC_cmd are §5.3 constant defaults
%             (EMS does not emit them in this scope). D_pu stays at the module
%             default 1.5; the grid-mode test OVERRIDES it to 60 at run time
%             (test-only, Rule 12 — real damping fix is Task 5.2b).
%
% Wiring groups (added in this order; the master prints per-group progress):
%   G1 DC star      : PV/Wind/ESS/EL/FC/VSC DC ports -> the 6 DCBus terminals
%   G2 AC chain     : Transmission -> PCC -> MGbus -> VSC (33/230 kV electrical)
%   G3 EMS commands : EMS dispatch -> ESS/Electrolyzer/PEMFC In_Pref_MW
%   G4 EMS inputs   : PV/Wind P -> EMS; P_load constant split -> EMS + MGbus
%   G5 sources      : profile constants (PV irr/T, Wind v, load Q), VSC refs, CB cmd
%   (H2 chain EL/FC->Tank is wired by the master step [6b]; left untouched.)

fprintf('  wire_microgrid: connecting the 12 modules...\n');

% Short paths
T   = [mdl '/TransmissionNetwork'];
PCC = [mdl '/PCC'];
BUS = [mdl '/MicrogridACBus'];
VSC = [mdl '/GridFormingVSC'];
DCB = [mdl '/DCBus'];
PV  = [mdl '/PV'];
WND = [mdl '/Wind'];
ESS = [mdl '/ESS'];
ELZ = [mdl '/Electrolyzer'];
FC  = [mdl '/PEMFC'];
EMS = [mdl '/EMS'];

% =====================================================================
%   G1 — DC STAR  (all six DCBus terminals tie to ONE node → order is
%   electrically irrelevant; each source module has exactly one DC port)
% =====================================================================
n = 0;
n = n + wire_elec(mdl, connByName(VSC,'Elec_DC_1500V'), connByName(DCB,'Elec_VSC_term'));
n = n + wire_elec(mdl, connByName(PV ,'Elec_DC_out'),   connByName(DCB,'Elec_PV_term'));
n = n + wire_elec(mdl, connByName(WND,'Elec_DC_out'),   connByName(DCB,'Elec_Wind_term'));
n = n + wire_elec(mdl, connByName(ESS,'Elec_DC_io'),    connByName(DCB,'Elec_ESS_term'));
n = n + wire_elec(mdl, connByName(ELZ,'Elec_DC_in'),    connByName(DCB,'Elec_Elec_term'));
n = n + wire_elec(mdl, connByName(FC ,'Elec_DC_out'),   connByName(DCB,'Elec_FC_term'));
fprintf('     G1 DC star      : %d/6 links\n', n);

% =====================================================================
%   G2 — AC CHAIN  Transmission(230) -> PCC -> MGbus(33) -> VSC
% =====================================================================
n = 0;
n = n + wire_elec(mdl, connByName(T  ,'Elec_PCC_230kV'), connByName(PCC,'Elec_HV_230kV'));
n = n + wire_elec(mdl, connByName(PCC,'Elec_MV_33kV'),   connByName(BUS,'Elec_PCC_in'));
n = n + wire_elec(mdl, connByName(BUS,'Elec_VSC_in'),    connByName(VSC,'Elec_AC_33kV'));
fprintf('     G2 AC chain     : %d/3 links\n', n);

% --- attach the SINGLE Solver Configuration to the now-unified network ---
% The master created SolverConfig + Gnd at top level but deferred wiring them.
% Now that all modules form ONE physical network, it needs exactly one Solver
% Configuration. Branch it onto a DC-bus node — a conserving port accepts
% multiple lines (forms a node); SolverConfig imposes NOTHING electrically, it
% only anchors the solver. (Do NOT branch the Electrical Reference here — that
% would short the +rail to ground; the modules carry their own grounds.)
sc = [mdl '/SolverConfig'];
if getSimulinkBlockHandle(sc) > 0
    phsc = get_param(sc,'PortHandles');
    hsc  = [phsc.LConn phsc.RConn];
    if ~isempty(hsc) && get_param(hsc(1),'Line') < 0
        add_line(mdl, hsc(1), connByName(DCB,'Elec_VSC_term'));
        fprintf('     SolverConfig    : attached to DC-bus node ✓\n');
    else
        fprintf('     SolverConfig    : already attached\n');
    end
end

% =====================================================================
%   G3 — EMS POWER COMMANDS  EMS -> ESS / Electrolyzer / PEMFC
% =====================================================================
n = 0;
n = n + wire_sig(mdl, sigPort(EMS,'Out_Pref_ess_MW','out'),  sigPort(ESS,'In_Pref_MW','in'));
n = n + wire_sig(mdl, sigPort(EMS,'Out_Pcmd_elec_MW','out'), sigPort(ELZ,'In_Pref_MW','in'));
n = n + wire_sig(mdl, sigPort(EMS,'Out_Pcmd_fc_MW','out'),   sigPort(FC ,'In_Pref_MW','in'));
fprintf('     G3 EMS commands : %d/3 links\n', n);

% =====================================================================
%   G4 — EMS POWER INPUTS  PV/Wind P -> EMS ; P_load constant -> EMS + MGbus
% =====================================================================
n = 0;
n = n + wire_sig(mdl, sigPort(PV ,'Out_P_pv_MW','out'),   sigPort(EMS,'In_P_pv_MW','in'));
n = n + wire_sig(mdl, sigPort(WND,'Out_P_wind_MW','out'), sigPort(EMS,'In_P_wind_MW','in'));
% single P_load constant SPLIT to the bus load AND the EMS net-power input (§3.13)
pf = 0.95;  Pld = 300;  Qld = Pld*tan(acos(pf));
sPload = add_const(mdl, 'Src_Pload_MW', Pld, [40 2040 100 2060]);
n = n + wire_sig(mdl, sPload, sigPort(BUS,'In_P_load_MW','in'));
n = n + wire_sig(mdl, sPload, sigPort(EMS,'In_P_load_MW','in'));   % branch
fprintf('     G4 EMS inputs   : %d/4 links (P_load=%g MW split bus+EMS)\n', n, Pld);

% =====================================================================
%   G5 — SOURCES  profile constants + bus Q + VSC refs + CB cmd
% =====================================================================
n = 0;
sQ   = add_const(mdl, 'Src_Qload_MVAr', Qld, [40 2090 100 2110]);
n = n + wire_sig(mdl, sQ, sigPort(BUS,'In_Q_load_MVAr','in'));
% RENEWABLE PLACEHOLDERS — now REALISTIC (PV 800 W/m² ≈52 MW + Wind 9 m/s ≈75 MW
% = 127 MW). Earlier these had to be throttled to ≈42 MW because the surplus ran
% V_dc away (the grid-forming VSC exported ≈0 real power at nominal f). The VSC
% DC-link voltage droop (build_vsc part A, 2026-06-28) is now the relief valve:
% surplus the ESS+electrolyzer cannot absorb raises V_dc slightly and the VSC
% exports it to the grid, so V_dc stays near 1500 V at full renewable output.
% Swap to From-Workspace real_profiles later.
%
% G9 WARM-START SOURCE RAMP (2026-07-02, see build_vsc.m G9 doc block for the
% full history: an earlier attempt forced the VSC sync state itself and
% diverged/failed to converge on the step-up transformer's zero-flux IC —
% reverted). Strategy here instead: ramp PV irradiance / wind speed linearly
% 0 -> nominal over t_src_ramp so total generation stays under the 80 MW ESS
% absorption ceiling until the UNTOUCHED G8 sync sequence completes (CB_sync
% closes ~0.457 s, export gate opens ~0.557 s): 127 MW * (0.557/1.0) ~= 71 MW
% < 75 MW target, leaving ESS headroom and no pre-existing surplus for the
% export droop to catch up on at sync completion. Reads the SAME
% base-workspace var `vsc_warm_start` (default TRUE) that build_vsc.m reads
% for its (informational-only) build summary. warm_start=FALSE -> renewables
% at full nominal from t=0 (matches the original black-start default).
try
    warm_start = evalin('base', 'vsc_warm_start');
catch
    warm_start = true;
end
t_src_ramp_s = 1.0;   % [s] ramp duration (see constraint above)
irrNom = 800; vNom = 9;   % nominal PV irradiance [W/m^2] / wind speed [m/s]
if warm_start
    sIrr = add_src_ramp(mdl, 'Src_PV_irr_Wm2', irrNom, t_src_ramp_s, [40 2140 100 2160]);
    sV   = add_src_ramp(mdl, 'Src_Wind_v_ms',  vNom,   t_src_ramp_s, [40 2240 100 2260]);
else
    sIrr = add_const(mdl, 'Src_PV_irr_Wm2', irrNom, [40 2140 100 2160]);
    sV   = add_const(mdl, 'Src_Wind_v_ms',  vNom,   [40 2240 100 2260]);
end
n = n + wire_sig(mdl, sIrr, sigPort(PV,'In_irr_Wm2','in'));
sT   = add_const(mdl, 'Src_PV_Tamb_C', 25,  [40 2190 100 2210]);
n = n + wire_sig(mdl, sT, sigPort(PV,'In_T_amb_C','in'));
n = n + wire_sig(mdl, sV, sigPort(WND,'In_v_wind_ms','in'));
sVr  = add_const(mdl, 'Src_VSC_Vref_pu', 1.0, [40 2290 100 2310]);  % §5.3 defaults
n = n + wire_sig(mdl, sVr, sigPort(VSC,'In_V_ref_pu','in'));
sFr  = add_const(mdl, 'Src_VSC_fref_Hz', 50,  [40 2340 100 2360]);
n = n + wire_sig(mdl, sFr, sigPort(VSC,'In_f_ref_Hz','in'));
sMd  = add_const(mdl, 'Src_VSC_mode', 1,      [40 2390 100 2410]);  % 1 = grid-forming (A2)
n = n + wire_sig(mdl, sMd, sigPort(VSC,'In_mode','in'));
sCB  = add_const(mdl, 'Src_CB_PCC_cmd', 1,    [40 2440 100 2460]);  % 1 = closed (grid mode)
n = n + wire_sig(mdl, sCB, sigPort(PCC,'In_CB_PCC_cmd','in'));
if warm_start
    fprintf('     G5 sources      : %d/8 links (PV/Wind %.0fs G9 ramp 0->800/9, Q=%.1f, VSC 1/50/1, CB=1)\n', n, t_src_ramp_s, Qld);
else
    fprintf('     G5 sources      : %d/8 links (PV 800/25, Wind 9, Q=%.1f, VSC 1/50/1, CB=1)\n', n, Qld);
end

fprintf('  wire_microgrid: done.\n');
end % wire_microgrid


% =====================================================================
%                              HELPERS
% =====================================================================
function h = connByName(ss, name)
% Outer Simscape connection-port handle for a named PMIOPort inside ss.
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
rank = find(nums==mynum, 1);
ph = get_param(ss,'PortHandles');
if strcmpi(side,'Left'), h = ph.LConn(rank); else, h = ph.RConn(rank); end
end

function h = sigPort(ss, name, dir)
% Outer Simulink signal-port handle for a named Inport/Outport inside ss.
blk = [ss '/' name];
k  = str2double(get_param(blk,'Port'));
ph = get_param(ss,'PortHandles');
if strcmp(dir,'in'), h = ph.Inport(k); else, h = ph.Outport(k); end
end

function ok = wire_elec(mdl, hA, hB)
% Connect two conserving ports if neither is already wired.
ok = 0;
if get_param(hA,'Line')>=0 || get_param(hB,'Line')>=0, return; end
add_line(mdl, hA, hB);  ok = 1;
end

function ok = wire_sig(mdl, hSrc, hDst)
% Connect a signal source port to a destination inport if the inport is free.
ok = 0;
if get_param(hDst,'Line')>=0, return; end       % dst already fed
add_line(mdl, hSrc, hDst, 'autorouting','on');  ok = 1;
end

function hOut = add_const(mdl, name, val, pos)
% Add (or reuse) a Constant block and return its output port handle.
blk = [mdl '/' name];
if getSimulinkBlockHandle(blk) <= 0
    add_block('simulink/Sources/Constant', blk, 'Value', num2str(val), ...
        'SampleTime','inf', 'Position', pos);
end
ph = get_param(blk,'PortHandles');
hOut = ph.Outport(1);
end

function hOut = add_src_ramp(mdl, name, nominal, t_ramp, pos)
% Add (or reuse) a Ramp(0->nominal over t_ramp)+Saturation(0,nominal) pair
% and return the Saturation block's output port handle (same call contract
% as add_const, so wire_sig callers don't care which one fed them). G9
% warm-start renewable source ramp — see build_vsc.m G9 doc block / the
% G5 comment above for why this exists (VSC-side IC forcing diverged).
rampBlk = [mdl '/' name '_ramp'];
satBlk  = [mdl '/' name];
if getSimulinkBlockHandle(satBlk) <= 0
    posR = pos - [90 0 90 0];
    add_block('simulink/Sources/Ramp', rampBlk, ...
        'Slope', num2str(nominal/t_ramp), 'Start', '0', ...
        'Position', posR);
    add_block('simulink/Discontinuities/Saturation', satBlk, ...
        'UpperLimit', num2str(nominal), 'LowerLimit', '0', ...
        'Position', pos);
    add_line(mdl, get_param(rampBlk,'PortHandles').Outport(1), ...
                  get_param(satBlk,'PortHandles').Inport(1), 'autorouting','on');
end
ph = get_param(satBlk,'PortHandles');
hOut = ph.Outport(1);
end
