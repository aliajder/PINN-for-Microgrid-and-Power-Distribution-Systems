function build_vsc(mdl)
%% build_vsc.m
% Builds the grid-forming VSC subsystem (GridFormingVSC) inside the master model.
%
% Reference : SIMULINK/IO_CONTRACTS.md §3.5 (GridFormingVSC) + CLAUDE.md A2/A8
% Master    : SIMULINK/build_microgrid_eelib.m calls this with the model name
% Params    : f0 and FixedStep read from base workspace / model (Rule 8/10)
%
% WHAT THIS MODULE IS (per IO_CONTRACTS §3.5):
%   Averaged 3-φ voltage-source converter with V-f droop (PRIMARY) and the
%   A8 SECONDARY restoration layer (slow PIs that drive f→f_ref and
%   V→V_ref in island mode). Grid-forming (A2): NO anti-islanding delay,
%   NO inner dq current loop (deferred per §3.5 note — modulation index is
%   driven directly by the droop output; adequate for f/power dynamics at
%   the averaged-model scope, R5).
%
%      DCBus ──~1500V~──►│ Elec_DC_1500V
%                        │  [Average-Value VSC 3φ]  v_ph = m·V_dc/2  (verified)
%                        │        │ LV 690 V (Yg)
%                        │  [XFMR 0.69/33 kV, Yg/Δ11, 350 MVA, 10% leak]
%                        │        │ MV 33 kV (3-wire)
%                        │  [CB_sync (G8)]──[I sens]──[V sens ∥]──► Elec_AC_33kV ──► MGbus
%                        │        │
%                        │  vf_droop (50 µs discrete): PQ calc+LPF → droop
%                        │  + A8 secondary PIs → θ,m_abc → modulation wave
%
% ── AV-VSC BLOCK DISCOVERY (probed 2026-06-10, R2026a) ────────────────────
%   ee_lib/Semiconductors & Converters/Converters/
%       Average-Value Voltage Source Converter (Three-Phase)
%   Verified port map : LConn(1)=PS modulation-wave input (3-vector),
%                       LConn(2)=3-φ AC terminal,
%                       RConn(1)=DC+, RConn(2)=DC−.
%   Verified gain law : v_phase = m · V_dc/2  exactly (m=0.8, Vdc=1500 →
%                       600.0 V measured). input_option=ModWave (default);
%                       set FRated=50, Vdc_nom=1500.
%
% ── DESIGN DECISIONS taken while building (2026-06-10) ────────────────────
%
%  G1. INTERNAL STEP-UP TRANSFORMER 690 V / 33 kV IS PHYSICALLY REQUIRED.
%      With V_dc = 1500 V the averaged law v=m·Vdc/2 caps the AC side at
%      750 V phase-peak (≈0.92 kV LL) — a 1500 V DC bus can NEVER make
%      33 kV directly. We use the utility-scale standard: converter LV
%      output 690 V LL (m̂ = 563/750 = 0.75 at 1.0 pu → 33% modulation
%      headroom), then a 350 MVA 0.69/33 kV step-up transformer.
%      The transformer's 10% leakage doubles as the COUPLING IMPEDANCE —
%      two voltage sources (grid via PCC XFMR, VSC) must never be
%      paralleled without impedance between them.
%
%  G2. VECTOR GROUP LV=Yg / MV=Δ11 (classic generator step-up). The MV
%      (33 kV) side is DELTA = 3-wire, so the VSC contributes NO ground
%      reference to the MG bus — exactly the assumption behind A9: the
%      zig-zag earthing transformer REMAINS the sole island-mode ground.
%      (A Yg MV side here would silently create a second ground source and
%      change the zero-sequence design.) LV Yg gives the converter-side
%      network its local reference.
%
%  G3. SOFT-START (t_soft = 0.2 s): the modulation command reaches 100% of
%      the droop target over the first 200 ms. Secondary integrators are
%      frozen during the ramp (no windup on a deliberately-wrong V).
%      ⚠ amended by G8: on an ENERGIZED bus the original 0→100% amplitude
%      ramp is NOT "small exchange" — it is a near-short (see G8).
%
%  G8. PRE-SYNCHRONIZATION on a live bus (added 2026-06-10, found by the
%      Task 5.6 ESS test). A ramping-from-zero VSC on an already-energized
%      33 kV bus is a ~0 V voltage source behind 10% leakage — a near-
%      bolted fault: MA-scale circulating current made the converter drain
%      the DC bus at ~80 MW; the 80 MW ESS saturated and V_dc collapsed
%      1500→155 V in 12 ms. (Task 5.5 never saw this because the TEST-ONLY
%      ideal DC source is infinitely stiff — it bankrolled the soft-start
%      short invisibly.) Phasor-mirroring without a breaker was tried and
%      is NOT enough: the t=0 network IC already solves INTO the fault
%      state (m_delay IC=0 ⇒ short), and the ~160 ms L/R fault-current
%      decay + the V_dc-adaptive division (m∝1/V_dc, floored at 750 V)
%      form a death spiral once V_dc sags. FINAL solution = the real-plant
%      connection sequence, a SYNC BREAKER (CB_sync, native ee_lib 3-φ
%      breaker as in build_pcc D1) between the step-up MV winding and the
%      bus port:
%        t=0: OPEN — the converter energizes only its own unloaded
%             transformer; the bus initializes healthy (no fault IC).
%        pre-sync: V/I sensors live on the BUS side; ramp the unloaded LV
%             to the measured |V| while a phase pull (k_sync=30 1/s,
%             τ≈33 ms) locks th to (φ_bus − π/6) (Yg/Δ11: MV leads LV
%             +30°). CLOSE when soft=1 AND |e_th| < 0.03 rad.
%        post-close: blend amplitude measured → droop command over
%             t_blend=0.1 s; A8 integrators unfreeze after the blend.
%      Dead bus (V_meas < 0.05 pu, island black start): close immediately
%      and energize via the plain G3 ramp.
%
%  G4. DROOP SLOPE NOTATION. §3.5 writes "f_cmd = f_ref − (1/R_p)·P_pu·f0".
%      Read literally with R_p=0.05 that is a gain of 20 → a 1000 Hz swing
%      at rated P — unphysical. The intended (and implemented) law is the
%      standard droop SLOPE form:  Δf = −R_p · P_pu · f0  (5% droop: rated
%      P → 2.5 Hz). The "1/R" form belongs to the inverse (governor
%      response) direction P = −(1/R)·Δf. Flagged in IO_CONTRACTS v1.6.
%
%  G5. P/Q FILTERING τ = 10 ms inside vf_droop (1st-order LPF on the
%      Clarke instantaneous P,Q before the droop law). Small-signal check
%      against the ~0.25 pu total coupling reactance gives the droop loop
%      poles ≈ −50 ± j60 (ζ≈0.6) — stable without an inner loop.
%
%  G6. V_dc-ADAPTIVE MODULATION: m is computed from MEASURED V_dc
%      (m = V_target_pk/(V_dc/2), guarded at ≥750 V). This linearises the
%      AC voltage against DC-bus sag AND is the natural §3.5 saturation
%      backstop: if V_dc collapses, m hits its 1.0 ceiling and the AC
%      voltage sags with it (no energy from nothing). The explicit
%      100 ms-timer backstop logic of §3.5 is deferred to the EMS task.
%
%  G7. GRID-FOLLOWING (mode=0) IS A STUB: f_cmd tracks f_meas, droop and
%      secondary are frozen, V_cmd = V_ref. Good enough as a placeholder —
%      this project runs the VSC grid-forming (mode=1, A2) throughout.
%
%  G9. WARM-START — STRATEGY: SOURCE RAMP, NOT VSC-SIDE IC FORCING (revised
%      2026-07-02). Original problem: at grid-mode default renewables
%      (PV 800 W/m² + Wind 9 m/s ≈137 MW) the G8 cold-sync sequence takes
%      ~0.55 s (soft-start 0.2s + phase-lock + 0.1s blend) before the DC-
%      link export droop can act, and the surplus overwhelms the 30 mF
%      C_dc before sync finishes.
%      FIRST ATTEMPT (reverted): force cb_state=1 / bypass the export gate
%      inside vf_droop so the VSC believes it is already synced from t=0.
%      Verified to FAIL: forcing the sync breaker closed (or the export
%      gate open) before the network's own initial condition is consistent
%      applies a voltage step onto the step-up transformer's zero-flux
%      initial state (classical energization inrush) — diverged with a
%      non-finite XFMR winding-flux derivative at t≈3.2 ms, and a follow-up
%      variant that only bypassed the export gate (leaving `conn`/A8 alone)
%      still failed Simscape's transient re-initialization at t≈3.75 ms.
%      This class of fix needs a physically-consistent transformer flux IC
%      (analogous to build_dcbus.m F2's capacitor `vc` IC), which depends
%      on the operating point and was not attempted here.
%      CURRENT STRATEGY: leave the G8 sync state machine COMPLETELY
%      UNTOUCHED (cb_state/t_close/conn below are exactly as originally
%      built — CB_sync closes ~0.457 s in, export opens ~0.557 s in, both
%      convergence-safe) and instead ramp the RENEWABLE SOURCES (PV
%      irradiance, wind speed) from zero up to nominal over the first
%      ~1 s, gated by the same `vsc_warm_start` base-workspace flag — see
%      wire_microgrid.m. This keeps total generation below the 80 MW ESS
%      absorption ceiling until sync completes (~0.557 s), so there is no
%      surplus for the export droop to catch up on, and it touches nothing
%      on the convergence-sensitive VSC/breaker/transformer path.
%      `warm_start` (read below, base-workspace var `vsc_warm_start`,
%      default TRUE) is kept here ONLY for the build summary fprintf —
%      wire_microgrid.m reads the SAME base-workspace var independently
%      for the actual source-ramp decision. No effect on the vf_droop
%      control law below (fully reverted to the original G8 form).
%
%  G10. FAZ 1+2 — INNER dq CURRENT LOOP (added 2026-07-04, doc/code only,
%      NO model run this session). Cascaded VSC control per Yazdani &
%      Iravani 2010 ("Voltage-Sourced Converters in Power Systems"): an
%      inner current loop (bandwidth-tuned PI + cross-coupling decoupling)
%      sits BELOW the existing V-f droop, on the ISLAND/grid-forming branch
%      only (mode>0.5). Gated by a build-time literal `inner_en`
%      (base-workspace `vsc_inner_loop_enable`, default FALSE): with it
%      false, vf_droop reproduces the ORIGINAL open-loop m synthesis
%      exactly (m_abc = m_abc_legacy) — bit-for-bit bypass, fully
%      reversible, no change to G1–G9 behavior.
%      Coupling impedance: the step-up transformer leakage (G1) IS the
%      only impedance between the VSC and the 33 kV MG bus — there is no
%      separate filter inductor. Referred to the 33 kV (MV) side (where
%      v_abc/i_abc are actually sensed, on the bus side of CB_sync):
%        Z_base_MV = 33kV²/350MVA = 3.111 Ω
%        L_coupling = Xleak_pu·Z_base_MV/(2π·50) ≈ 0.991 mH
%        R_coupling = 2·Rw_pu·Z_base_MV          ≈ 0.0124 Ω
%      Current-loop bandwidth target α_c = 2π·500 rad/s (pole-zero
%      cancellation, Kp_i=α_c·L≈3.11, Ki_i=α_c·R≈39.1 — 33 kV-side A/V
%      units). i_d, i_q use the SAME amplitude-invariant Clarke/Park
%      convention as the existing P/Q calc, i.e. PEAK-amplitude scale
%      (matches v_abc/i_abc directly), NOT rms — current limit i_max_pk is
%      converted from an rms rating accordingly (see literal block below).
%      Current reference for this phase (id*=P_ref/(1.5vd), iq*=-Q_ref/
%      (1.5vd)) is a SCAFFOLD: P_ref/Q_ref are the droop's own P_f/Q_f
%      measurement filter outputs, NOT yet an outer V_dc-regulation loop
%      (that is Faz 3). id gets priority in current limiting; the legacy
%      open-loop V_eff/m_pk/G8 sync state machine is left COMPLETELY
%      UNTOUCHED (still computed every step) so the bypass is a true no-op
%      and G8 pre-sync/black-start is unaffected regardless of inner_en.
%
%  G10a. INIT-CONVERGENCE FIX (added 2026-07-04, after the first inner-loop
%      run failed transient init at t≈0.156 s = the conn/inner_active
%      handoff). Two coupled corrections, BOTH inside the inner branch only
%      (legacy path and Kp_i/Ki_i/limit untouched):
%      (1) TRANSFORMER TURNS RATIO in the modulation. v_abc/i_abc are sensed
%          on the 33 kV MV bus (bus side of CB_sync), so the whole loop is
%          MV-referred (v_d, i_d, L_c, the current limit — all consistent).
%          But the AVVSC produces the LV (690 V) voltage and the 0.69/33 kV
%          step-up (G1) makes the MV. So an MV command vd_star is realised
%          by an LV terminal voltage vd_star/n_t, i.e. m = 2·vd_star/(n_t·Vdc),
%          n_t = V_mv_LL/V_lv_LL ≈ 47.83. The original m = 2·vd_star/Vdc
%          omitted n_t → m ≈ 36 → clamped to 1 → the converter was grossly
%          over-driven; that over-drive, applied as a STEP at the handoff, is
%          what failed the DAE re-init. (Legacy already refers correctly —
%          its Vph_pk = 563 V is the LV phase peak, no n_t needed there.)
%      (2) BUMPLESS TRANSFER at the rising edge of inner_active: back-
%          calculate id_int/iq_int once so vd_star/vq_star equal the legacy
%          converter voltage (n_t·V_cmd_pu·Vph_pk on d, 0 on q) that same
%          step → m_abc_inner == m_abc_legacy exactly at the handoff, no
%          voltage step across the leakage. After that step the integrators
%          evolve normally. This also gives the integrators a physically
%          consistent starting value at the operating point (they never run
%          from a cold 0 into a live bus). The algebraic loop is already
%          broken by the existing m_delay Unit Delay (m_abc reaches the AVVSC
%          one step later); no extra measurement-side delay is needed.
%
%  G10b. FAZ 3 — CURRENT-LIMITED GRID-FORMING (added 2026-07-04, after the
%      Faz-1+2 island run: with the inner loop ALWAYS active the VSC behaved
%      as a current source, NOBODY formed the island frequency, f drifted to
%      53.76 Hz and V_dc sagged). Rocabert et al. 2012: an islanded grid-
%      forming VSC must remain the VOLTAGE/frequency former; the inner current
%      loop is only a fault/overcurrent LIMITER. So the inner loop is no longer
%      scheduled on `conn` — it ARMS on overcurrent and disarms on recovery:
%        - trigger: a hysteresis latch ilim_on — set when |i_dq| > i_max_pk,
%          cleared when |i_dq| < ilim_exit (= 0.95*i_max_pk). Anti-chatter
%          (Red-Flag "state machine oscillation -> hysteresis").
%        - normal operation (not over-limit): w_blend = 0 -> m_abc is PURE
%          m_abc_legacy -> the droop (§3.5) + A8 secondary form V and f. This
%          is the fix for the 53.76 Hz drift.
%        - over-limit: inner_active=1, w_blend=1 -> m_abc = m_abc_inner. The
%          current reference is the MEASURED current clamped onto the i_max_pk
%          circle (direction preserved), so the PI pulls |i| down to i_max.
%          The G10a bumpless preload still fires at the RISING edge so the
%          entry into limiting is step-free (no DAE re-init failure).
%        - release: w_blend decays 0 over t_ib (5 ms); during the decay the
%          inner PI tracks the PRESENT current (identity reference, integrators
%          frozen) so its phasor stays coherent and converges to legacy -> the
%          handback blends smoothly with no voltage step.
%      inner_en (base-workspace vsc_inner_loop_enable) now ARMS this limiter;
%      FALSE = pure legacy voltage-forming (bit-for-bit, the limiter never
%      engages). The Kp_i/Ki_i/decoupling/n_t of G10/G10a are unchanged — only
%      the ENGAGEMENT logic (trigger + reference + blend) changed.

% ── Parameters (no magic numbers; units in names) ─────────────────────────
f0_Hz      = evalin('base','f0');                    % 50 Hz (Rule 10)
dt_s       = str2double(get_param(mdl,'FixedStep')); % 50 µs — droop sample time

S_vsc_VA   = 350e6;   % VSC rating: PV80+Wind120+ESS80+FC30 ≈ 310 MW + margin [VA]
V_lv_LL    = 690;     % converter LV line-line RMS              [V]   (G1)
V_mv_LL    = 33e3;    % MG bus line-line RMS                    [V]   (A3)
V_dc_nom_V = 1500;    % DC bus nominal                          [V]   (A1)
Xleak_pu   = 0.10;    % step-up XFMR total leakage = coupling Z [pu]  (G1)
Rw_pu      = 0.002;   % XFMR winding resistance                 [pu]

R_p        = 0.05;    % P-f droop slope (5%: rated P → 2.5 Hz)  [pu]  % TUNED: placeholder (§3.5)
R_q        = 0.05;    % Q-V droop slope (5%: rated Q → 0.05 pu) [pu]  % TUNED: placeholder (§3.5)
tau_PQ_s   = 0.010;   % P/Q measurement LPF                     [s]   (G5)

% A8 secondary restoration layer — slow PIs (seconds), an order below the
% ms-scale droop so the loops separate cleanly (Guerrero hierarchy).
Kp_f       = 0.1;     % f-restoration proportional   [Hz/Hz]   % TUNED: placeholder
Ki_f       = 0.5;     % f-restoration integral       [1/s]     % TUNED: placeholder
df_max_Hz  = R_p*f0_Hz;  % restoration trim clamp [Hz] = FULL primary-droop authority
                         % (R_p·f0 = 2.5 Hz). FIX 2026-07-05: was 0.5 Hz, but at heavy
                         % ISLAND load the droop sag exceeds it (e.g. 210 MW → sag =
                         % R_p·(210/350)·f0 = 1.5 Hz), so the secondary SATURATED and f
                         % stuck at ~49.0 Hz < 49.2 Hz (the EMS EMERGENCY |Δf|≤0.8 exit)
                         % → permanent EMERGENCY latch. Sizing the clamp to the primary
                         % droop authority lets the integrator cancel the offset at any
                         % load up to rated → f→50, the latch self-clears. Still a hard
                         % Rule-7 anti-windup clamp; grid mode unaffected (err_f≈0 there).
Kp_V       = 0.5;     % V-restoration proportional   [pu/pu]   % TUNED: placeholder
Ki_V       = 2.0;     % V-restoration integral       [1/s]     % TUNED: placeholder
dV_max_pu  = 0.10;    % restoration trim clamp       [pu]      (Rule 7)

% DC-link voltage droop (A, 2026-06-28) — grid-mode "relief valve": export DC-bus
% surplus to the grid when V_dc rises above nominal. Holds V_dc near 1500 V so the
% renewable power the ESS (±80) + electrolyzer cannot absorb does NOT run V_dc away
% (the grid-mode finding, IO_CONTRACTS §4.1 / R1/R4). Active grid-connected only.
V_dc_band_V    = 30;     % droop band: 1500→0 export, 1500+band→full export [V]  % TUNED (A)
P_export_max_W = 100e6;  % grid-export cap (PCC 300 MVA XFMR headroom)       [VA] % TUNED (A)

% FAZ 1+2 (2026-07-04) — inner dq current-loop parameters (G10). Coupling Z
% = the step-up XFMR leakage referred to the 33 kV (MV) side — no separate
% filter inductor exists in this design.
Z_base_MV_ohm  = V_mv_LL^2 / S_vsc_VA;                        % = 3.111 Ω
L_coupling_H   = (Xleak_pu * Z_base_MV_ohm) / (2*pi*f0_Hz);   % ≈ 0.991 mH
R_coupling_ohm = (2*Rw_pu) * Z_base_MV_ohm;                    % ≈ 0.0124 Ω
alpha_c_radps  = 2*pi*500;               % current-loop bandwidth target: 500 Hz
Kp_i           = alpha_c_radps * L_coupling_H;    % ≈ 3.11  [V/A]  (pole-zero cancel)
Ki_i           = alpha_c_radps * R_coupling_ohm;  % ≈ 39.1  [V/(A·s)]
I_rms_max_A    = 1.1 * S_vsc_VA / (sqrt(3)*V_mv_LL);  % ≈ 6736 A rms (110% rated)
i_max_pk_A     = sqrt(2) * I_rms_max_A;               % ≈ 9527 A PEAK — id/iq are
                                                       % amplitude-invariant PEAK-scale
                                                       % (same convention as v_abc/i_abc),
                                                       % NOT rms; this converts the rms
                                                       % nameplate rating to that scale.
% FAZ 3 (2026-07-04, G10b) — the inner loop is repurposed from an always-on
% current source into an OVERCURRENT-triggered current LIMITER (Rocabert 2012:
% in island the VSC must stay VOLTAGE-forming and form f; the inner loop only
% caps current on a transient). Hysteresis + soft handback avoid chatter.
ilim_exit_frac = 0.95;   % exit current-limiting below 0.95*i_max (enter @ i_max) — anti-chatter
t_ilim_blend_s = 0.005;  % soft handback (inner -> legacy) blend time on exit [s]

% G-INNER: base-workspace flag `vsc_inner_loop_enable` (FAZ3 default should be
% TRUE — the current LIMITER is a protection layer you normally want ON; it is
% inert until an overcurrent). FALSE fully bypasses it (pure legacy voltage-
% forming, bit-for-bit). Read at BUILD time and baked into the MATLAB Function
% body as a literal, same pattern as `warm_start` above.
try
    inner_loop_en = evalin('base', 'vsc_inner_loop_enable');
catch
    inner_loop_en = false;   % default: bypassed, G1–G9 behavior unchanged
end

% NOTE (2026-07-03): the "decouple V_dc from f by disabling df_export in grid
% mode, leaving the ESS PI as the sole fast V_dc regulator" experiment was tried
% and REVERTED (Rule 12). At the PHYSICAL D_pu=1.5 it made V_dc VIOLENTLY
% unstable — a 1 kV↔34 kV oscillation from the moment the VSC syncs (~0.47 s),
% NOT a slow drift and NOT triggered by a load step. Reason: df_export is the
% VSC's ONLY DC-bus relief path (V_dc→angle→export); remove it and the ESS
% (±80 MW) alone cannot hold the DC bus against the undamped ~2–3 Hz mode.
% See RESULTS/vdc_decoupled.mat + SIMULINK/vdc_decoupled_test.m. The real fix is
% JOINT: real ~2–3 Hz damping (Task 5.2b) + a proper grid-following V_dc OUTER
% loop (Teodorescu 2011) — not disabling the export path. Export droop stays ON.

m_max      = 1.0;     % linear modulation ceiling (no overmodulation)
t_soft_s   = 0.2;     % amplitude soft-start ramp               [s]   (G3)
t_blend_s  = 0.1;     % post-close amplitude blend              [s]   (G8)

% G9: base-workspace var `vsc_warm_start` (default TRUE), read here ONLY for
% the build summary fprintf below. The actual warm-start behavior (a
% renewable-source ramp) is implemented independently in wire_microgrid.m,
% which reads the same base-workspace var — see the G9 doc block above.
try
    warm_start = evalin('base', 'vsc_warm_start');
catch
    warm_start = true;    % default (G9)
end

Vph_pk_LV  = V_lv_LL * sqrt(2/3);   % 690 V LL → 563.4 V phase peak (1.0 pu target)

% ── Subsystem container ───────────────────────────────────────────────────
ssName = 'GridFormingVSC';
ssPath = [mdl '/' ssName];
if getSimulinkBlockHandle(ssPath) > 0
    Simulink.SubSystem.deleteContents(ssPath);
else
    add_block('built-in/Subsystem', ssPath, 'Position', [200 820 360 960]);
    Simulink.SubSystem.deleteContents(ssPath);
end

% =====================================================================
%   BOUNDARY PORTS
% =====================================================================
dcPort = [ssPath '/Elec_DC_1500V'];
add_block('built-in/PMIOPort', dcPort, 'Side', 'Left',  'Position', [40 300 60 320]);
acPort = [ssPath '/Elec_AC_33kV'];
add_block('built-in/PMIOPort', acPort, 'Side', 'Right', 'Position', [1400 300 1420 320]);

add_block('built-in/Inport',  [ssPath '/In_V_ref_pu'],   'Position', [40 80 60 98]);
add_block('built-in/Inport',  [ssPath '/In_f_ref_Hz'],   'Position', [40 130 60 148]);
add_block('built-in/Inport',  [ssPath '/In_mode'],       'Position', [40 180 60 198]);
add_block('built-in/Outport', [ssPath '/Out_P_vsc_MW'],  'Position', [1400 80 1420 98]);
add_block('built-in/Outport', [ssPath '/Out_Q_vsc_MVAr'],'Position', [1400 130 1420 148]);
add_block('built-in/Outport', [ssPath '/Out_V_dc_V'],    'Position', [1400 180 1420 198]);
% FAZ 1+2 (G10) diagnostics — always present (id/iq computed regardless of
% inner_en); id_ref/iq_ref read 0 when the inner loop is not active.
add_block('built-in/Outport', [ssPath '/Out_i_d_A'],     'Position', [1400 420 1420 438]);
add_block('built-in/Outport', [ssPath '/Out_i_q_A'],     'Position', [1400 460 1420 478]);
add_block('built-in/Outport', [ssPath '/Out_i_d_ref_A'], 'Position', [1400 500 1420 518]);
add_block('built-in/Outport', [ssPath '/Out_i_q_ref_A'], 'Position', [1400 540 1420 558]);

% =====================================================================
%   AVERAGE-VALUE VSC  (verified: LConn1=mod-wave PS, LConn2=AC 3φ,
%   RConn1=DC+, RConn2=DC−;  v_phase = m·V_dc/2)
% =====================================================================
vsc = [ssPath '/AVVSC'];
add_block(av_vsc_source(), vsc, 'Position', [340 280 440 380]);
set_param(vsc, 'FRated', num2str(f0_Hz), 'Vdc_nom', num2str(V_dc_nom_V));

% DC side: + rail to boundary, − rail to ground (build_dcbus F1 convention:
% grounded-negative 1500 V bus; the G2 transformer isolates this from the
% 33 kV AC grounding, so A9's zig-zag remains the island AC ground).
add_line(ssPath, conn(dcPort,'RConn',1), conn(vsc,'RConn',1));
add_block('ee_lib/Connectors & References/Electrical Reference', [ssPath '/Gnd_DC'], ...
    'Position', [340 440 380 470]);
add_line(ssPath, conn(vsc,'RConn',2), conn([ssPath '/Gnd_DC'],'LConn',1));

% Local DC voltage measurement (sensor ∥ DC port): LConn1=+, RConn1=PS, RConn2=−.
vdcs = [ssPath '/V_dc_sens'];
add_block('ee_lib/Sensors & Transducers/Voltage Sensor', vdcs, 'Position', [200 380 260 440]);
add_line(ssPath, conn(dcPort,'RConn',1), conn(vdcs,'LConn',1));
add_line(ssPath, conn(vdcs,'RConn',2), conn([ssPath '/Gnd_DC'],'LConn',1));
add_block('nesl_utility/PS-Simulink Converter', [ssPath '/Vdc_PS2S'], 'Position', [200 480 240 510]);
set_param([ssPath '/Vdc_PS2S'], 'Unit', 'V');
add_line(ssPath, conn(vdcs,'RConn',1), conn([ssPath '/Vdc_PS2S'],'LConn',1));
add_line(ssPath, 'Vdc_PS2S/1', 'Out_V_dc_V/1', 'autorouting','on');

% =====================================================================
%   STEP-UP TRANSFORMER 0.69 / 33 kV  (Yg LV / Δ11 MV — G1/G2)
% =====================================================================
xf = [ssPath '/XFMR_069_33'];
add_block('ee_lib/Passive/Transformers/Two-Winding Transformer (Three-Phase)', xf, ...
    'Position', [560 280 660 380]);
set_param(xf, ...
    'SRated',  num2str(S_vsc_VA), ...
    'FRated',  num2str(f0_Hz), ...
    'Winding1Connection', 'ee.enum.windingconnection.Yg', ...      % LV = Yg (G2)
    'VRated1', num2str(V_lv_LL), ...
    'Winding2Connection', 'ee.enum.windingconnection.delta11', ... % MV = Δ, 3-wire (G2/A9)
    'VRated2', num2str(V_mv_LL), ...
    'leakage_reactance_option', 'ee.enum.transformer_leakage.include', ...
    'pu_Xl1',  num2str(Xleak_pu/2), 'pu_Xl2', num2str(Xleak_pu/2), ...
    'pu_Rw1',  num2str(Rw_pu),      'pu_Rw2', num2str(Rw_pu));
add_line(ssPath, conn(vsc,'LConn',2), conn(xf,'LConn',1));   % AV-VSC AC → LV winding

% =====================================================================
%   MV MEASUREMENTS — I (series) + V (parallel) at the 33 kV boundary.
%   Current orientation XFMR→bus, so +P = injecting into AC bus (§1.1).
% =====================================================================
% G8 sync breaker between the MV winding and the bus port (same native
% block + inverted-control semantics as build_pcc D1: ctrl>0.5 = OPEN).
% The V/I sensors sit on the BUS side so pre-sync can read the live grid
% while the breaker is open.
cbx = [ssPath '/CB_sync'];
add_block('ee_lib/Switches & Breakers/Circuit Breaker (Three-Phase)', cbx, ...
    'Position', [690 280 740 360]);
set_param(cbx, ...
    'breaker_behavior', 'ee.enum.switches.breakerBehavior.zeroCrossingEnable', ...
    'threshold', '0.5', 'R_closed', '1e-3', 'G_open', '1e-6');
add_line(ssPath, conn(xf,'RConn',1), conn(cbx,'LConn',2));        % MV winding → CB in

isens = [ssPath '/I_vsc'];
add_block('ee_lib/Sensors & Transducers/Current Sensor (Three-Phase)', isens, ...
    'Position', [760 280 820 360]);
add_line(ssPath, conn(cbx,'RConn',1),   conn(isens,'LConn',1));   % CB out → I in
add_line(ssPath, conn(isens,'RConn',2), conn(acPort,'RConn',1));  % I out → AC boundary

vsens = [ssPath '/V_vsc'];
add_block('ee_lib/Sensors & Transducers/Phase Voltage Sensor (Three-Phase)', vsens, ...
    'Position', [900 380 960 450]);
add_block('ee_lib/Connectors & References/Grounded Neutral (Three-Phase)', [ssPath '/V_vsc_ref'], ...
    'Position', [900 500 960 530]);
add_line(ssPath, conn(isens,'RConn',2), conn(vsens,'LConn',1));
add_line(ssPath, conn(vsens,'RConn',2), conn([ssPath '/V_vsc_ref'],'LConn',1));

add_block('nesl_utility/PS-Simulink Converter', [ssPath '/I_PS2S'], 'Position', [860 200 900 230]);
add_block('nesl_utility/PS-Simulink Converter', [ssPath '/V_PS2S'], 'Position', [1000 395 1040 425]);
add_line(ssPath, conn(isens,'RConn',1), conn([ssPath '/I_PS2S'],'LConn',1));
add_line(ssPath, conn(vsens,'RConn',1), conn([ssPath '/V_PS2S'],'LConn',1));

% =====================================================================
%   vf_droop — PRIMARY droop + A8 SECONDARY restoration + θ/m_abc synthesis
%   One discrete (50 µs) MATLAB Function: PQ calc/LPF, droop law, slow
%   restoration PIs with Rule-7 anti-windup, soft-start, modulation wave.
% =====================================================================
drp = [ssPath '/vf_droop'];
add_block('simulink/User-Defined Functions/MATLAB Function', drp, 'Position', [620 60 780 200]);
set_mlfcn(drp, { ...
 'function [m_abc, f_cmd_Hz, P_vsc_MW, Q_vsc_MVAr, cb_open, i_d_A, i_q_A, i_d_ref_A, i_q_ref_A] = vf_droop(v_abc, i_abc, V_dc_V, f_meas_Hz, V_meas_pu, V_ref_pu, f_ref_Hz, mode, dt_s, f0_Hz, CB_grid)'
 '%#codegen'
 '% Grid-forming V-f droop + A8 secondary restoration (IO_CONTRACTS §3.5).'
 '% FAZ 1+2 (2026-07-04, G10): optional inner dq current loop, gated by'
 '% inner_en below. Default inner_en=0 reproduces the legacy open-loop m'
 '% synthesis EXACTLY (bit-for-bit bypass).'
 '% Persistent discrete states: oscillator phase, P/Q filters, secondary'
 '% integrators, soft-start clock, saturation flag, inner current-loop PIs.'
 ['S_base_VA = ' num2str(S_vsc_VA) ';    % droop power base = VSC rating']
 ['R_p   = ' num2str(R_p)    ';  R_q   = ' num2str(R_q) ';   % droop SLOPES (G4: §3.5 "(1/R)" read as slope)']
 ['tauPQ = ' num2str(tau_PQ_s) ';                  % P/Q LPF [s] (G5)']
 ['Kp_f  = ' num2str(Kp_f)  ';  Ki_f  = ' num2str(Ki_f)  ';  df_max = ' num2str(df_max_Hz) ';  % A8 f-restore + clamp']
 ['Kp_V  = ' num2str(Kp_V)  ';  Ki_V  = ' num2str(Ki_V)  ';  dV_max = ' num2str(dV_max_pu) ';  % A8 V-restore + clamp']
 ['Vph_pk = ' num2str(Vph_pk_LV,'%.6g') ';  Vdc_nom = ' num2str(V_dc_nom_V) ';  m_lim = ' num2str(m_max) ';']
 ['t_soft = ' num2str(t_soft_s) ';                 % amplitude ramp [s] (G3)']
 ['t_blend = ' num2str(t_blend_s) ';                % post-close amplitude blend [s] (G8)']
 ['Kdc_WpV = ' num2str(P_export_max_W/V_dc_band_V,'%.6g') ';  P_exp_max_W = ' num2str(P_export_max_W,'%.6g') ';  % DC-link export droop (A)']
 ['Kp_i = ' num2str(Kp_i,'%.6g') ';  Ki_i = ' num2str(Ki_i,'%.6g') ';  % FAZ1+2 inner current PI (33kV side, G10)']
 ['L_c = ' num2str(L_coupling_H,'%.6g') ';  R_c = ' num2str(R_coupling_ohm,'%.6g') ';  % coupling L[H]/R[Ohm] = XFMR leakage (G1), no separate filter']
 ['n_t = ' num2str(V_mv_LL/V_lv_LL,'%.6g') ';  % step-up XFMR turns ratio MV/LV (G10a init fix): AVVSC=LV, sensing=MV']
 ['i_max_pk = ' num2str(i_max_pk_A,'%.6g') ';  % current limit, PEAK amps (id/iq are peak-scale, amplitude-invariant)']
 ['ilim_exit = ' num2str(ilim_exit_frac,'%.6g') '*i_max_pk;  t_ib = ' num2str(t_ilim_blend_s,'%.6g') ';  % FAZ3 (G10b) limit hysteresis-lo + exit blend [s]']
 ['inner_en = ' num2str(double(inner_loop_en)) ';  % FAZ3 enable: 0=pure legacy voltage-forming, 1=overcurrent current-limiter armed']
 '    persistent th P_f Q_f int_f int_V t_run sat_prev cb_state t_close id_int iq_int inner_prev ilim_on w_blend'
 '    if isempty(th)'
 '        % G8: start the LV oscillator ON the measured bus phase so the'
 '        % pre-sync pull converges fast (Yg/D11: MV leads LV +30°).'
 '        th = atan2((1/sqrt(3))*(v_abc(2)-v_abc(3)), ...'
 '                   (2/3)*(v_abc(1)-0.5*v_abc(2)-0.5*v_abc(3))) - pi/6;'
 '        P_f=0; Q_f=0; int_f=0; int_V=0; t_run=0; sat_prev=0;'
 '        cb_state=0; t_close=0;              % G8: sync breaker starts OPEN'
 '        id_int=0; iq_int=0; inner_prev=0;   % FAZ1+2: inner current-loop PI states + edge flag (G10a)'
 '        ilim_on=0; w_blend=0;               % FAZ3 (G10b): overcurrent latch + legacy<->inner blend weight'
 '    end'
 ''
 '    % ── Instantaneous P/Q at the 33 kV terminal (amplitude-invariant Clarke)'
 '    va=v_abc(1); vb=v_abc(2); vc=v_abc(3); ia=i_abc(1); ib=i_abc(2); ic=i_abc(3);'
 '    valpha=(2/3)*(va-0.5*vb-0.5*vc);  vbeta=(1/sqrt(3))*(vb-vc);'
 '    ialpha=(2/3)*(ia-0.5*ib-0.5*ic);  ibeta=(1/sqrt(3))*(ib-ic);'
 '    P_W   = 1.5*(valpha*ialpha + vbeta*ibeta);   % + = injecting into AC bus'
 '    Q_VAR = 1.5*(vbeta*ialpha - valpha*ibeta);'
 '    P_f = P_f + (P_W   - P_f)*dt_s/tauPQ;        % 1st-order measurement LPF'
 '    Q_f = Q_f + (Q_VAR - Q_f)*dt_s/tauPQ;'
 '    Vdc_eff = max(V_dc_V, 0.5*Vdc_nom);          % FAZ1+2: moved up, needed before the oscillator section'
 ''
 '    % ── Soft-start amplitude ramp (G3); integrators frozen until the'
 '    % sync breaker is closed AND the post-close blend is done (G8).'
 '    t_run = t_run + dt_s;'
 '    soft  = min(t_run/t_soft, 1);'
 '    conn  = (cb_state > 0.5) && (t_run - t_close >= t_blend);'
 ''
 '    if mode > 0.5   % ─── GRID-FORMING (A2: no anti-islanding delay) ───'
 '        % A8 secondary restoration: slow PIs trim f_ref/V_ref so the'
 '        % steady state returns to the setpoints despite the droop offset.'
 '        err_f = f_ref_Hz - f_meas_Hz;'
 '        err_V = V_ref_pu - V_meas_pu;'
 '        % Rule 7 anti-windup: integrate only when the modulator was NOT'
 '        % saturated last step and the unit is CONNECTED + blended (G8).'
 '        if sat_prev < 0.5 && conn'
 '            int_f = int_f + Ki_f*err_f*dt_s;'
 '            int_V = int_V + Ki_V*err_V*dt_s;'
 '        end'
 '        int_f = max(-df_max, min(df_max, int_f));    % clamp (Rule 7)'
 '        int_V = max(-dV_max, min(dV_max, int_V));'
 '        df_restore = max(-df_max, min(df_max, Kp_f*err_f + int_f));'
 '        dV_restore = max(-dV_max, min(dV_max, Kp_V*err_V + int_V));'
 ''
 '        % DC-LINK VOLTAGE DROOP (A): export DC-bus surplus to the grid. When'
 '        % V_dc rises above nominal (ESS saturated on surplus renewables), bias'
 '        % the droop frequency UP so the VSC angle leads the grid and real power'
 '        % flows OUT, pulling V_dc back. df_export = R_p*f0*(P_export/S) makes the'
 '        % converter settle at P=P_export (steady state f_cmd=grid f). Grid-'
 '        % connected ONLY (CB_grid) AND synced (conn); island has no grid → 0.'
 '        df_export = 0;'
 '        if CB_grid > 0.5 && conn'
 '            P_exp_W = Kdc_WpV*(V_dc_V - Vdc_nom);'
 '            if P_exp_W < 0, P_exp_W = 0; end'
 '            if P_exp_W > P_exp_max_W, P_exp_W = P_exp_max_W; end'
 '            df_export = R_p*f0_Hz*(P_exp_W/S_base_VA);'
 '        end'
 '        % PRIMARY droop (slope form, G4): rated P → R_p·f0 Hz sag.'
 '        f_cmd_Hz = (f_ref_Hz + df_restore + df_export) - R_p*(P_f/S_base_VA)*f0_Hz;'
 '        V_cmd_pu = (V_ref_pu + dV_restore) - R_q*(Q_f/S_base_VA);'
 '    else            % ─── GRID-FOLLOWING (stub, G7) ───'
 '        f_cmd_Hz = f_meas_Hz;        % track the grid; droop/secondary off'
 '        V_cmd_pu = V_ref_pu;'
 '    end'
 ''
 '    % ══════════════════════════════════════════════════════════════════'
 '    % FAZ 1+2 (G10) — INNER dq CURRENT LOOP, island/grid-forming path only.'
 '    % Yazdani & Iravani 2010 cascaded VSC control. th used below is THIS'
 '    % sample angle (BEFORE the oscillator update further down) — the same'
 '    % instant the v_abc/i_abc measurements were taken.'
 '    % ══════════════════════════════════════════════════════════════════'
 '    c_th = cos(th); s_th = sin(th);'
 '    i_d_A = ialpha*c_th + ibeta*s_th;   % Park (amplitude-invariant -> PEAK'
 '    i_q_A = -ialpha*s_th + ibeta*c_th;  % scale, matches i_abc directly, NOT rms)'
 '    % ══ FAZ 3 (G10b): GRID-FORMING with CURRENT LIMITING (Rocabert 2012). The'
 '    % VSC stays VOLTAGE-forming (m_abc_legacy forms f) in NORMAL operation; the'
 '    % inner dq loop engages ONLY on OVERCURRENT, as a current LIMITER. It is NOT'
 '    % an always-on current source (that let nobody form f -> f drifted to 53.76'
 '    % Hz). Hysteresis latch: enter at i_max, leave below ilim_exit (0.95*i_max).'
 '    i_mag = sqrt(i_d_A*i_d_A + i_q_A*i_q_A);'
 '    if i_mag > i_max_pk,  ilim_on = 1; end   % over-limit -> arm the limiter'
 '    if i_mag < ilim_exit, ilim_on = 0; end   % recovered  -> release (hysteresis)'
 '    inner_active = double((mode > 0.5) && (inner_en > 0.5) && conn && (ilim_on > 0.5));'
 '    % Output blend weight w_blend: jumps to 1 while limiting (the G10a bumpless'
 '    % preload keeps the ENTRY step-free), decays to 0 over t_ib on release (soft'
 '    % handback to legacy voltage-forming). inner_compute keeps the inner PI/phasor'
 '    % coherent through the decay window so the blend has a live target.'
 '    if inner_active > 0.5'
 '        w_blend = 1;'
 '    else'
 '        w_blend = max(0, w_blend - dt_s/t_ib);'
 '    end'
 '    inner_compute = (inner_active > 0.5) || (w_blend > 1e-3);'
 '    if inner_compute'
 '        v_d = valpha*c_th + vbeta*s_th;'
 '        v_q = -valpha*s_th + vbeta*c_th;'
 '        % FAZ 3 current reference = the MEASURED current direction clamped onto'
 '        % the i_max_pk circle. Over-limit: scl<1 pulls |i| down to i_max (id/iq'
 '        % share preserved -- the protective action). Blend-out (not over-limit):'
 '        % scl=1 = identity -> the inner voltage tracks the present operating'
 '        % point and converges to legacy, so the handback is smooth.'
 '        if inner_active > 0.5 && i_mag > 1e-6'
 '            scl = i_max_pk / i_mag;'
 '        else'
 '            scl = 1;'
 '        end'
 '        i_d_ref_A = i_d_A * scl;'
 '        i_q_ref_A = i_q_A * scl;'
 '        % Decoupled PI (Yazdani & Iravani form).'
 '        e_id = i_d_ref_A - i_d_A;  e_iq = i_q_ref_A - i_q_A;'
 '        omega_c = 2*pi*f_meas_Hz;'
 '        if inner_active > 0.5 && inner_prev < 0.5'
 '            % G10a BUMPLESS TRANSFER at the RISING edge of current-limiting:'
 '            % preload integrators so vd_star/vq_star equal the LEGACY converter'
 '            % voltage this very step -- d = n_t*V_cmd_pu*Vph_pk (MV-referred),'
 '            % q = 0 -- making m_abc_inner == m_abc_legacy at the handoff (no'
 '            % voltage step across the leakage inductance).'
 '            id_int = n_t*V_cmd_pu*Vph_pk - v_d + omega_c*L_c*i_q_A - Kp_i*e_id;'
 '            iq_int = -v_q - omega_c*L_c*i_d_A - Kp_i*e_iq;'
 '        elseif inner_active > 0.5'
 '            % actively limiting: integrate (circle reference is always feasible,'
 '            % and limiting lowers m -> no ceiling windup, Rule 7 satisfied).'
 '            id_int = id_int + Ki_i*e_id*dt_s;'
 '            iq_int = iq_int + Ki_i*e_iq*dt_s;'
 '        end'
 '        % (blend-out: integrators FROZEN at their release value; identity ref'
 '        %  gives e~0 so vd_star holds a coherent phasor that decays into legacy.)'
 '        vd_star = v_d - omega_c*L_c*i_q_A + Kp_i*e_id + id_int;'
 '        vq_star = v_q + omega_c*L_c*i_d_A + Kp_i*e_iq + iq_int;'
 '        valpha_s = vd_star*c_th - vq_star*s_th;   % inverse Park'
 '        vbeta_s  = vd_star*s_th + vq_star*c_th;'
 '        % G10a: /n_t refers the MV-side command back through the step-up'
 '        % transformer to the LV converter terminal before m = 2*v_conv/Vdc.'
 '        m_abc_inner = (2/(n_t*Vdc_eff)) * [ valpha_s; ...                   % inverse Clarke'
 '                                      -0.5*valpha_s + (sqrt(3)/2)*vbeta_s; ...'
 '                                      -0.5*valpha_s - (sqrt(3)/2)*vbeta_s ];'
 '    else'
 '        i_d_ref_A = 0;  i_q_ref_A = 0;  m_abc_inner = [0;0;0];'
 '    end'
 '    inner_prev = double(inner_active);   % G10a/b: rising-edge tracker for the bumpless preload'
 ''
 '    % ── Oscillator + modulation wave.  v_phase = m·V_dc/2 (verified), so'
 '    % m = V_target_pk/(V_dc/2) with MEASURED V_dc (G6 backstop: V_dc sag'
 '    % → m ceiling → AC sags too; no energy from nothing).'
 '    % ── G8 SYNC BREAKER + pre-sync state machine. Behind the OPEN'
 '    % breaker the converter only magnetizes its own unloaded step-up,'
 '    % so the t=0 m=0 instant cannot bolt a fault on the live bus. The'
 '    % bus-side sensors keep reading the live grid: ramp the unloaded LV'
 '    % to the measured |V| while k_sync locks th to (φ_bus − 30°), then'
 '    % CLOSE on phase match and blend amplitude to the droop command.'
 '    k_sync  = 30;                            % phase pull gain [1/s]'
 '    phi_bus = atan2(vbeta, valpha);'
 '    e_th    = atan2(sin(phi_bus - pi/6 - th), cos(phi_bus - pi/6 - th));'
 '    if cb_state < 0.5                        % ── OPEN: pre-synchronize'
 '        % dead-bus close needs t_run>0.05 s: the V_pcc measurement reads'
 '        % ~0 for the first samples (discrete startup), which must not be'
 '        % mistaken for a dead bus (it closed the breaker at 100 us once).'
 '        if V_meas_pu < 0.05 && t_run > 0.05'
 '            cb_state = 1;  t_close = t_run;  % dead bus: black start'
 '        elseif soft >= 1 && abs(e_th) < 0.03'
 '            cb_state = 1;  t_close = t_run;  % matched: connect'
 '        end'
 '        th    = th + (2*pi*f_cmd_Hz + k_sync*e_th)*dt_s;'
 '        V_eff = soft * V_meas_pu;            % land on the bus magnitude'
 '    else                                     % ── CLOSED'
 '        th = th + 2*pi*f_cmd_Hz*dt_s;'
 '        if V_meas_pu > 0.05'
 '            mu    = min((t_run - t_close)/t_blend, 1);'
 '            V_eff = (1-mu)*V_meas_pu + mu*V_cmd_pu;  % soft handoff'
 '        else'
 '            V_eff = soft * V_cmd_pu;         % black start: plain G3 ramp'
 '        end'
 '    end'
 '    th = mod(th, 2*pi);'
 '    m_pk = V_eff * Vph_pk / (Vdc_eff/2);'
 '    if m_pk > m_lim'
 '        m_pk = m_lim;  sat_prev = 1;     % freeze integrators next step'
 '    else'
 '        sat_prev = 0;'
 '    end'
 '    m_abc_legacy = m_pk * [cos(th); cos(th - 2*pi/3); cos(th + 2*pi/3)];'
 ''
 '    % FAZ 3 selection (G10b): blend legacy voltage-forming <-> inner current-'
 '    % limiter by w_blend. w_blend=0 (normal, no overcurrent) -> PURE legacy so'
 '    % the VSC forms f; w_blend=1 (limiting) -> pure inner so |i| is clamped to'
 '    % i_max; the decay is the soft handback. inner_en=0 keeps ilim_on/w_blend at'
 '    % 0 -> legacy only, bit-for-bit bypass preserved.'
 '    m_sel = (1 - w_blend)*m_abc_legacy + w_blend*m_abc_inner;'
 '    m_abc = max(-m_lim, min(m_lim, m_sel));'
 ''
 '    P_vsc_MW   = P_f/1e6;                % filtered, + = injecting (§1.1)'
 '    Q_vsc_MVAr = Q_f/1e6;'
 '    cb_open    = 1 - cb_state;           % G8: breaker ctrl (>0.5 = OPEN)'
 'end'});
% Persistent states → discrete sample time mandatory (same as the PCC PLL).
cfgD = get_param(drp, 'MATLABFunctionConfiguration');
cfgD.UpdateMethod = 'Discrete';
cfgD.SampleTime   = num2str(dt_s);

% ── droop inputs ──
% 1 v_abc, 2 i_abc: MV sensors. 3 V_dc. 4 f_meas, 5 V_pcc: global Froms
% (two of the five legal Goto tags). 6-8 boundary refs. 9 dt, 10 f0.
add_block('simulink/Signal Routing/From', [ssPath '/From_f'], ...
    'GotoTag','f_meas', 'Position', [480 100 540 118]);
add_block('simulink/Signal Routing/From', [ssPath '/From_V'], ...
    'GotoTag','V_pcc',  'Position', [480 130 540 148]);
add_block('simulink/Signal Routing/From', [ssPath '/From_CB'], ...
    'GotoTag','CB_PCC', 'Position', [480 220 540 238]);   % grid-connected flag (A: DC-link droop gate)
add_block('built-in/Constant', [ssPath '/dt_c'], 'Value', num2str(dt_s),  'Position', [480 160 530 180]);
add_block('built-in/Constant', [ssPath '/f0_c'], 'Value', num2str(f0_Hz), 'Position', [480 190 530 210]);

add_line(ssPath, 'V_PS2S/1',     'vf_droop/1', 'autorouting','on');
add_line(ssPath, 'I_PS2S/1',     'vf_droop/2', 'autorouting','on');
add_line(ssPath, 'Vdc_PS2S/1',   'vf_droop/3', 'autorouting','on');
add_line(ssPath, 'From_f/1',     'vf_droop/4', 'autorouting','on');
add_line(ssPath, 'From_V/1',     'vf_droop/5', 'autorouting','on');
add_line(ssPath, 'In_V_ref_pu/1','vf_droop/6', 'autorouting','on');
add_line(ssPath, 'In_f_ref_Hz/1','vf_droop/7', 'autorouting','on');
add_line(ssPath, 'In_mode/1',    'vf_droop/8', 'autorouting','on');
add_line(ssPath, 'dt_c/1',       'vf_droop/9', 'autorouting','on');
add_line(ssPath, 'f0_c/1',       'vf_droop/10','autorouting','on');
add_line(ssPath, 'From_CB/1',    'vf_droop/11','autorouting','on');

% ── droop outputs ──
% m_abc → Unit Delay (1 sample) → Simulink-PS → AV-VSC mod-wave input.
% The UNIT DELAY breaks the algebraic loop sensors→vf_droop→m_abc→network→
% sensors (m_abc is direct feedthrough from the same-step V/I measurements;
% a 1st-order S-PS input filter does NOT break it because the filter
% derivative x'=(u−x)/τ still depends directly on u). One 50 µs sample is
% also the honest model of a real controller's computation delay.
% The S-PS filter (τ=1e-4 s) stays to give the DAE a smooth derivative;
% its 1.8° phase lag at 50 Hz is absorbed by the droop equilibrium.
add_block('simulink/Discrete/Unit Delay', [ssPath '/m_delay'], ...
    'SampleTime', num2str(dt_s), 'Position', [320 180 350 210]);
add_block('nesl_utility/Simulink-PS Converter', [ssPath '/m_S2PS'], 'Position', [380 180 420 210]);
set_param([ssPath '/m_S2PS'], 'FilteringAndDerivatives', 'filter', ...
    'SimscapeFilterOrder', '1', 'InputFilterTimeConstant', '1e-4');
add_line(ssPath, 'vf_droop/1', 'm_delay/1', 'autorouting','on');
add_line(ssPath, 'm_delay/1',  'm_S2PS/1',  'autorouting','on');
add_line(ssPath, conn([ssPath '/m_S2PS'],'RConn',1), conn(vsc,'LConn',1));
% f_cmd is a droop-internal diagnostic (terminate; scope it in tests if needed).
add_block('built-in/Terminator', [ssPath '/f_cmd_term'], 'Position', [820 100 840 120]);
add_line(ssPath, 'vf_droop/2', 'f_cmd_term/1', 'autorouting','on');
add_line(ssPath, 'vf_droop/3', 'Out_P_vsc_MW/1',   'autorouting','on');
add_line(ssPath, 'vf_droop/4', 'Out_Q_vsc_MVAr/1', 'autorouting','on');
% FAZ 1+2 (G10) diagnostics: id/iq always live; id_ref/iq_ref read 0 unless
% the inner loop is active (mode>0.5 && inner_en && conn).
add_line(ssPath, 'vf_droop/6', 'Out_i_d_A/1',     'autorouting','on');
add_line(ssPath, 'vf_droop/7', 'Out_i_q_A/1',     'autorouting','on');
add_line(ssPath, 'vf_droop/8', 'Out_i_d_ref_A/1', 'autorouting','on');
add_line(ssPath, 'vf_droop/9', 'Out_i_q_ref_A/1', 'autorouting','on');
% G8 sync-breaker control: Unit Delay (same algebraic-loop hygiene as m_abc)
% → filtered S-PS → CB_sync PS control (>0.5 = OPEN; fed directly as cb_open).
% G9 note: this sync state machine is UNCHANGED (see the G9 doc block above —
% the warm-start strategy is now a source-side ramp in wire_microgrid.m, not
% VSC-side IC forcing; an earlier attempt to force this breaker/gate closed
% at t=0 was reverted after it failed to converge — see G9 for the detail).
add_block('simulink/Discrete/Unit Delay', [ssPath '/cb_delay'], ...
    'SampleTime', num2str(dt_s), 'InitialCondition', '1', ...   % start OPEN (G8)
    'Position', [820 150 850 180]);
% NO input filter here: the filter STATE initializes at 0, so for the
% first ~τ the control would read < 0.5 = CLOSED — re-creating the very
% t=0 fault IC the breaker exists to prevent. The control is a threshold
% compare (no derivative needed) → 'provide' with zero derivative.
add_block('nesl_utility/Simulink-PS Converter', [ssPath '/cb_S2PS'], 'Position', [880 150 920 180]);
set_param([ssPath '/cb_S2PS'], 'FilteringAndDerivatives', 'provide');
add_line(ssPath, 'vf_droop/5', 'cb_delay/1', 'autorouting','on');
add_line(ssPath, 'cb_delay/1', 'cb_S2PS/1',  'autorouting','on');
add_line(ssPath, conn([ssPath '/cb_S2PS'],'RConn',1), conn(cbx,'LConn',1));

fprintf('  build_vsc: AV-VSC (m·Vdc/2) + 0.69/33 kV Yg/Δ11 step-up + vf_droop (primary V-f + A8 secondary + DC-link export droop). G8 sync unchanged; warm_start=%d source-ramp strategy lives in wire_microgrid.m. FAZ3 current-LIMITED grid-forming inner_en=%d (G10b: legacy voltage-forms f, inner dq loop arms only on overcurrent >%.0f A pk, hysteresis exit %.0f%%).\n', warm_start, inner_loop_en, i_max_pk_A, ilim_exit_frac*100);
end % build_vsc


% =====================================================================
%                              HELPERS
% =====================================================================
function src = av_vsc_source()
% The library block name contains embedded newlines ("Average-Value\n
% Voltage Source\nConverter\n(Three-Phase)") — resolve it by regexp so the
% add_block source path is exact regardless of the wrapping.
if ~bdIsLoaded('ee_lib'), load_system('ee_lib'); end
cand = find_system('ee_lib', 'LookUnderMasks','all', 'Regexp','on', ...
    'Name', 'Average-Value\s+Voltage Source\s+Converter\s+\(Three-Phase\)');
assert(~isempty(cand), 'Average-Value Voltage Source Converter (Three-Phase) not found in ee_lib.');
src = cand{1};
end

function h = conn(blkPath, side, idx)
% Conserving/PS port handle by side ('LConn'/'RConn') and index.
ph = get_param(blkPath, 'PortHandles');
h  = ph.(side)(idx);
end

function set_mlfcn(blkPath, lines)
% Set a MATLAB Function block body via the modern configuration API.
% NOTE: never route through sprintf — '%' chars (e.g. %#codegen) would be
% interpreted as format specifiers and mangled.
cfg = get_param(blkPath, 'MATLABFunctionConfiguration');
cfg.FunctionScript = strjoin(lines, newline);
end
