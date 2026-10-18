# IO_CONTRACTS.md — Microgrid Simulink Model I/O Contracts (v1.15)

> **Read this BEFORE editing any subsystem.** Every block's inputs/outputs are
> defined here. If you change a port, update this file in the same commit.
>
> Companion files: CLAUDE.md (v4), PROJECT_KNOWLEDGE.md (v2).

---

## 0. SIGNAL-TYPE TAXONOMY

Two categories. Never mix them on the same line.

| Category | Notation | Block colour | Direction | Carrier |
|---|---|---|---|---|
| **Electrical** | `~Elec~` | Simscape physical (cyan, dashed if 3-phase) | Bidirectional | Voltage AND current together; obeys Kirchhoff |
| **Control / measurement** | `→Sim→` | Simulink directed signal (black, solid arrow) | Single-direction | Scalar (or bus) value only |

**Rule:** Electrical ports connect with Simscape lines. Control signals connect with Simulink arrows. The only legal bridge is the `Simulink-PS Converter` / `PS-Simulink Converter` pair (`nesl_utility/…`). All such bridges are documented per subsystem.

---

## 1. UNIT CONVENTIONS — CRITICAL

**The model uses SI on the wire and per-unit only inside controllers.** This is the convention; pu/SI mismatches in signal lines have caused every previous model bug in this project.

| Quantity class | Wire unit | When pu is used | Conversion point |
|---|---|---|---|
| Active power P | **MW** | inside swing-eq and droop math | `P_W = P_MW * 1e6` at PS converter input |
| Reactive power Q | **MVAr** | inside V-Q droop math | `Q_VAR = Q_MVAr * 1e6` at PS converter input |
| Absolute frequency | **Hz** | nowhere on wire | — |
| Δf (deviation) | **Hz** | inside droop: `df_pu = df / f0` | inside MATLAB Function only |
| Setpoint AC voltage | **kV** RMS L-L | inside V droop: `v_pu = v_kV / 33` (MG) or `/230` (txm) | inside MATLAB Function only |
| Measured AC voltage | **pu** of local base | always | computed by Phase Voltage Sensor + Gain |
| DC voltage | **V** | rarely; if pu needed, base = 1500 V | inside controllers |
| Current | **A** | rarely | — |
| SOC (battery, H2) | **0–1 dimensionless ratio** | always (no conversion needed) | — |
| Inertia H | **s** | always | — |
| Damping D, droop 1/R | **pu/pu dimensionless** | always | — |
| Time constants τ | **s** | always | — |
| Mass (H2) | **kg** | — | — |
| Mass flow | **kg/s** | — | — |
| Pressure (tank) | **bar** | — | — |
| Temperature | **°C** | — | — |
| Wind speed | **m/s** | — | — |
| Boolean flags | **0/1** (uint8) | — | — |

### 1.1 Sign conventions

| Signal | Positive direction |
|---|---|
| `P_ess_*_MW` | **+ = discharge** (battery → DC bus), **− = charge** |
| `P_fc_MW` | **+ = generate** only (PEMFC cannot consume); range [0, +P_fc_max] |
| `P_elec_MW` | **+ = consume** (electrolyzer is a load); range [0, +P_elec_max] |
| `P_pv_MW`, `P_wind_MW` | **+ = generate** only |
| `P_vsc_MW` (AC side) | **+ = injecting AC bus** (DC→AC), **− = absorbing** (AC→DC) |
| `P_pcc_MW` | **+ = MG importing** from transmission |
| `mdot_H2_*` | **+ for production** (electrolyzer out), **+ for consumption** (FC in) — stored as two separate signals, never combined to one |
| `CB_PCC` | **1 = closed**, **0 = open** |
| `VSC_mode` | **1 = grid-forming**, **0 = grid-following** |

### 1.2 Where conversion happens (the only 4 places)

1. **Reference power → Simscape source block input.** A controlled current source on the DC bus expects W. Multiply by 1e6 in a Gain block immediately before the Simulink-PS Converter.
2. **Voltage measurement → control signal.** Phase Voltage Sensor (Three-Phase) outputs phase voltage in V. A Gain `1/Vbase_kV/1000 * sqrt(3)` produces line-line pu. Wrapped in a small `V_to_pu` subsystem (one per AC node).
3. **Inside MATLAB Function blocks** that implement droop / swing equations — variables divided by f0, P_base etc. at the top of the function. Always in physical units on the boundary.
4. **EMS output stage.** EMS computes references in MW directly; no scaling.

**There is no other place where pu/SI conversion may occur. If you find yourself converting elsewhere, stop — the wiring is wrong.**

---

## 2. GLOBAL SIGNALS (Goto / From)

**Only the five signals below may use Goto/From.** Everything else must be a direct port-to-port Simulink line. This rule is non-negotiable; it keeps the model readable when ee_lib block names already take up half the canvas.

| Tag name | Type | Unit | Source block (Goto owner) | Used by (From consumers) |
|---|---|---|---|---|
| `f_meas` | →Sim→ | Hz | PCC subsystem (PLL or VSC reference, see §5.2) | EMS, ESS droop, FC droop, scopes |
| `SOC_ESS` | →Sim→ | 0–1 | ESS subsystem | EMS, scopes |
| `SOC_H2` | →Sim→ | 0–1 | H2Tank subsystem | EMS, scopes |
| `V_pcc` | →Sim→ | pu (base 33 kV) | PCC subsystem (V_to_pu) | EMS, scopes |
| `CB_PCC` | →Sim→ | 0/1 (uint8) | PCC subsystem (breaker status output) | EMS, VSC mode logic, scopes |

**Tag visibility:** all five are `Global` scope. Goto block colour: green (Simulink default for global).

---

## 3. SUBSYSTEM CONTRACTS

Naming convention for ports:
- Electrical ports: `Elec_<role>` (e.g., `Elec_AC_33kV`, `Elec_DC_1500V`)
- Control inputs: `In_<name>_<unit>` (e.g., `In_Pref_MW`)
- Measurement outputs: `Out_<name>_<unit>` (e.g., `Out_Pact_MW`)

Each subsystem table has columns:
- **Port** — block-edge label
- **Dir** — In/Out/Bidirectional
- **Type** — Elec or Sim
- **Unit** — see §1
- **Source/Dest** — where it comes from / goes to
- **Notes** — sign, range, default, implementation hint

---

### 3.1 `TransmissionNetwork`

**Role:** case5 230 kV transmission system. Holds Bus 1/3/4/5, six branches, Bus 3 and Bus 4 loads.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_PCC_230kV` | Bidirectional | Elec (3-φ) | kV / A | → `PCC` subsystem | Connection to Bus 2 |
| `Out_V_buses_pu` | Out | Sim (bus, 4×1) | pu | → diagnostic scope (optional) | [V_b1, V_b3, V_b4, V_b5], base 230 kV |
| `Out_f_buses_Hz` | Out | Sim (bus, 3×1) | Hz | → diagnostic scope (optional) | [f_b1, f_b3, f_b5] from ClassicalSM ω/2π |

**Internal structure:** 1× ideal Voltage Source (Three-Phase) for Bus 4 (slack), 3× ClassicalSM submodules for Bus 1/3/5, 6× RLC (Three-Phase) for branches, 2× Wye-Connected Load (Bus 3 = 300 MW, Bus 4 = 400 MW). All parameters from `mg.bus`, `mg.gen`, `mg.branch`.

**Parameters (read from `mg_params.mat` at model init):**
- `mg.branch.R_ohm(k)`, `mg.branch.L_henry(k)`, `mg.branch.C_farad(k)` for k=1..6
- `mg.bus.Pd_MW`, `mg.bus.Qd_MVAr` for loads at Bus 3 and Bus 4
- `mg.gen.Pg_MW`, `mg.gen.Vg_pu` for Bus 1/3/5 SMs

**PI-section construction (verified 2026-05-27):** `mg.branch.C_farad(k)` stores the **full lumped line capacitance** in Farad, magnitude **~40–190 nF** (= 0.04–0.19 µF) for case5 at 230 kV. For each PI section, the shunt capacitance is split as **C/2 at each end** of the series R–L branch. Each branch therefore uses **three RLC (Three-Phase) blocks**:
| Position | Block config | Values |
|---|---|---|
| Series R–L | `port_option = 'Series RLC'`, R/L/C ports | `R = mg.branch.R_ohm(k)`, `L = mg.branch.L_henry(k)`, `C = Inf` |
| Shunt C at "from" end | `port_option = 'Delta'` (line-to-line) or `'Wye with neutral'`, shunt to Electrical Reference | `C = mg.branch.C_farad(k)/2`, `R = Inf`, `L = 0` |
| Shunt C at "to" end | same as above | same value |

Sanity: Q_charging at 230 kV computed = 0.66 → 3.13 MVAr per branch, consistent with `B_pu` × V²/S_base.

**CRITICAL build parameter — slack source (verified 2026-05-28):** the Bus 4 `Voltage Source (Three-Phase)` MUST set `impedance_option='0'` (ideal/stiff). The library **default `'1'` adds a huge internal impedance that collapses a loaded bus to ~0.1 pu** — this cost hours of debugging. `vline_rms` is line-to-line RMS in volts (`230e3`). Source/CVS/sensor port maps (PS vs electrical-conserving ports are indistinguishable by `PortType` — all read `connection`): VS3 → LConn(1)=neutral(1φ), RConn(1)=`~`(3φ). CVS3 → LConn(1)=PS ref, LConn(2)=neutral(1φ), RConn(1)=`~`. Phase Voltage Sensor → LConn(1)=`~` node, RConn(1)=PS out, RConn(2)=`~` ref. Current Sensor → LConn(1)=`~` in, RConn(1)=PS out, RConn(2)=`~` out. Wye-Connected Load → LConn(1)=`~`(3φ), RConn(1)=neutral(1φ).

**Validation result (build_transmission.m v1 vs `runpf(case5)`, 2026-05-28):** steady state matches load-flow — slack (Bus 4) 1.0000 pu exact, all bus angles within **±0.41°**, magnitudes **+0.6 to +2.7%** (no-AVR classical-model residual, largest at Bus 2/3). Settled with elevated transient damping (D=30); D does not change the steady state. **Solver caveat:** validated under implicit fixed-step (ode1be/ode14x @50µs) and ode23t; explicit **ode4 @50µs diverges** on this ideal-source stiff network — Rule 1 conformance is an OPEN architectural item (see SIMULINK/NEXT_STEPS.md §5.2a).

---

### 3.2 `ClassicalSM` (template, used 3× — one per Bus 1/3/5)

**Role:** 2nd-order synchronous machine model (swing equation + back-EMF as 3-φ controlled source). No AVR, no governor.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_Term_230kV` | Bidirectional | Elec (3-φ) | kV / A | → `TransmissionNetwork` bus node | Machine terminals |
| `In_Pm_MW` | In | Sim | MW | parameter constant (no governor v1) | Mechanical power input; default = `mg.gen.Pg_MW(this_bus)` |
| `Out_f_Hz` | Out | Sim | Hz | → `TransmissionNetwork/Out_f_buses_Hz` | f = f0 + Δf |
| `Out_Pe_MW` | Out | Sim | MW | scope / diagnostic | Electrical power computed from V·I |
| `Out_delta_rad` | Out | Sim | rad | scope / diagnostic | Rotor angle |

**Internal blocks:**
- Current Sensor (3-φ) at terminals → P_e computation (MATLAB Function: `Pe = 3·real(V·conj(I))`)
- MATLAB Function `swing_eq` implementing `2H·dω/dt = (Pm - Pe)/S_base_machine - D·Δω_pu`
- ω integrator → δ integrator
- Controlled Voltage Source (3-φ) driven by `V_set_pu·cos(ω·t + δ)` (3-phase generator)

**Parameters (per instance):**
- `H_s` (default 5.5 s for transmission SMs)
- `D_pu` (default 1.5)
- `S_base_MW` = machine rating (Pg_MW)
- `V_set_pu` = `mg.gen.Vg_pu(this_bus)` (default 1.00)
- `f0_Hz` = 50 (from base workspace)

**Bug guard:** Use `tf([1],[2H, D])` swing form, NOT `tf([ws],[2H, D*ws])`. See CLAUDE.md Rule 2.

---

### 3.3 `PCC` (Point of Common Coupling)

**Role:** Step-up transformer 33/230 kV (Δ-Y, ~300 MVA), main breaker CB1, frequency and voltage measurement.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_HV_230kV` | Bidirectional | Elec (3-φ) | kV / A | ↔ `TransmissionNetwork/Elec_PCC_230kV` | HV side of XFMR |
| `Elec_MV_33kV` | Bidirectional | Elec (3-φ) | kV / A | ↔ `MicrogridACBus/Elec_PCC_in` | MV side, downstream of CB1 |
| `In_CB_PCC_cmd` | In | Sim | 0/1 | ← `EMS/Out_CB_PCC_cmd` | 1=close, 0=open |
| `Out_f_meas_Hz` | Out (Goto) | Sim | Hz | → Goto `f_meas` (global) | PLL on MV side (see §5.2) |
| `Out_V_pcc_pu` | Out (Goto) | Sim | pu (base 33 kV) | → Goto `V_pcc` (global) | Phase Voltage Sensor + V_to_pu |
| `Out_CB_PCC_status` | Out (Goto) | Sim | 0/1 | → Goto `CB_PCC` (global) | Mirror of cmd input (no delay yet) |
| `Out_P_pcc_MW` | Out | Sim | MW | scope / EMS | Active flow across XFMR, + = MG importing |
| `Out_Q_pcc_MVAr` | Out | Sim | MVAr | scope / EMS | Reactive flow |

**Internal blocks (AS BUILT — build_pcc.m v1, 2026-06-06):**
- **Two-Winding Transformer (Three-Phase):** 300 MVA, **HV 230 kV = Δ (delta11), MV 33 kV = Yg**, leakage 10% (pu_Xl1=pu_Xl2=0.05), pu_Rw=0.002. See vector-group decision **D2** below.
- **Breaker:** the **native `ee_lib/Switches & Breakers/Circuit Breaker (Three-Phase)`** (the earlier "ee_lib has no 3-φ breaker → build a 3× switch composite" assumption was WRONG — the block exists). Verified port map (probed 2026-06-06): **LConn(1)=PS control, LConn(2)=3-φ electrical IN, RConn(1)=3-φ electrical OUT**. `breaker_behavior=zeroCrossingEnable` (opens at next current zero → suppresses the L·di/dt spike). Control semantics: **control > 0.5 → OPEN; < 0.5 → CLOSED** — the OPPOSITE of our `CB_PCC` convention, so build_pcc feeds the breaker **`ctrl = 1 − In_CB_PCC_cmd`** (a Constant−Sum inverter).
- **Phase Voltage Sensor (3-φ)** on the MV node (after CB1) → `V_to_pu` (MATLAB Function, amplitude-invariant Clarke magnitude → LL-RMS pu, base 33 kV) → Goto `V_pcc`.
- **PLL:** SRF-PLL `PLL_freq` (MATLAB Function, persistent θ + PI integral; Clarke → normalise → q-axis error → PI → θ). **Runs on a DISCRETE 50 µs sample time** (`UpdateMethod='Discrete'`) — persistent state is illegal on a continuous/inherited rate. Gains Kp=180, Ki=3200 (placeholders). → Goto `f_meas`.
- **P/Q:** `PQ_calc` (MATLAB Function, Clarke-based `P=1.5(vα iα+vβ iβ)`, `Q=1.5(vβ iα−vα iβ)`) from the MV V & I (series Current Sensor on the breaker→MV line). Sign: + current = transmission→MG, so **+P = MG importing**. → ports Out_P_pcc_MW / Out_Q_pcc_MVAr.

**Validated (build_pcc_test.m, 2026-06-06, TransmissionNetwork+PCC, ode14x@50µs, 50 MW MV test load):**
CLOSED → P_pcc=+51.6 MW, V_pcc=1.016 pu, f_meas=50.21 Hz. OPEN → P_pcc=0, V_pcc=0.007 pu (MG isolated; no VSC yet). Transformer ratio, breaker, sign, and PLL all confirmed.

**D2 — vector-group decision (FLAG for advisor):** CLAUDE.md A3 reads "33/230 Δ-Y" (33 kV=Δ); this §3.3 historically read "230/33 Δ-Y" (230 kV=Δ). build_pcc resolves the conflict as **HV 230 kV=Δ, MV 33 kV=Yg** because the 33 kV MG bus needs a solid ground reference for V measurement / PLL / common mode BEFORE the grid-forming VSC exists (a Δ MV side would float); the HV side is already grounded by the TransmissionNetwork. This is the textbook substation step-down group (Dyn). ⚠ If A3's literal "33 kV=Δ" is mandatory, the MicrogridACBus must supply its own MV ground reference (grounding transformer). A 30° Δ-Y phase shift between HV and MV is physical/expected.

**Single-network note (2026-06-06):** the transformer's magnetic coupling keeps HV+MV as ONE Simscape "physical network" — only ONE Solver Configuration is allowed for the whole model (a second on the MV side errors). The Yg secondary supplies the MV ground reference.

**Numerical stability note — breaker opening:** `zeroCrossingEnable` opening is the primary L·di/dt mitigation. An **RC snubber** (`CB1_Snubber`, RLC 3-φ Series RC across the breaker poles, R_sn=200 Ω, C_sn=1 µF — `% TUNED:` placeholders) is also fitted per the original guidance; tune during the Template B (islanding) test. The standalone test showed no solver issues on the open transition.

---

### 3.4 `MicrogridACBus`

**Role:** 33 kV common 3-phase bus, local 300 MW variable load, interface to PCC and VSC.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_PCC_in` | Bidirectional | Elec (3-φ) | kV / A | ↔ `PCC/Elec_MV_33kV` | From upstream |
| `Elec_VSC_in` | Bidirectional | Elec (3-φ) | kV / A | ↔ `GridFormingVSC/Elec_AC_33kV` | From microgrid VSC |
| `In_P_load_MW` | In | Sim | MW | ← From Workspace (`real_profiles.P_load`) | Scaled to 300 MW peak from ENTSO-E TR |
| `In_Q_load_MVAr` | In | Sim | MVAr | derived: `Q = P·tan(acos(pf))`, `pf = mg.bus.pf(bus2)` | Default pf = 0.95 lagging |
| `Out_I_bus_A` | Out | Sim (3×1) | A | scope (optional) | Total bus current |

**Internal blocks (AS BUILT — build_mg_acbus.m v1, 2026-06-08):**
- **Load:** the **`Wye-Connected Variable Load (lagging)`** (component `wye_series_RL`), NOT the plain `Wye-Connected Variable Load` — the plain one is **resistive-only** (`wye_R`, P with no Q). The lagging variant takes BOTH demands as physical-signal inputs. **Verified port map (probed 2026-06-08): `LConn(1)=P demand [W]` (PS), `LConn(2)=Q demand [VAr]` (PS), `LConn(3)=3-φ ~ terminal`, `RConn(1)=neutral`.** P/Q arrive on the boundary in MW/MVAr and are scaled ×1e6 → W/VAr at Simulink-PS converters (units `W` / `V*A`). Load neutral → 1-φ Electrical Reference.
- **Behaviour:** the realised power is **constant-impedance** (the block sets R-L to draw the commanded P,Q at *rated* voltage; at off-nominal voltage power follows V²). Confirmed: 300 MW/98 MVAr at 33 kV exactly, scaling to ~268 MW at 0.945 pu in the system test.
- **Bus current sensor:** 3-φ Current Sensor in **series on the PCC feed** (Elec_PCC_in → sensor → bus node), so the bus node = its electrical-OUT port; the load, the zig-zag, and Elec_VSC_in all branch off that node. → `Out_I_bus_A`.
- **Elec_VSC_in** is exposed (bus node tied to the boundary port) but **left open at top level until Task 5.5** (no VSC yet).

**Validated (build_mgbus_test.m, 2026-06-08, TransmissionNetwork+PCC+MicrogridACBus, CB1 CLOSED, ode14x@50µs):** V_pcc=**0.9448 pu** (within grid-code [0.9,1.1]; droop expected — no VSC/reactive support yet, 300 MVA XFMR @10% leakage carries ~285 MVA), P_pcc=**+267.8 MW** (importing; = constant-Z load at 0.945 pu), f_meas=**49.97 Hz** stable, zig-zag I_neutral≈**0 A**. The transmission capacitor-IC relax warning is the known pre-existing quirk (also in build_pcc_test); non-fatal.

**Grounding (NEW v1.4, CLAUDE.md A9):** the `ZigZagGrounding` transformer (§3.4.1) is **permanently connected to this 33 kV bus** to supply an island-mode ground reference that survives CB1 opening. **As built it is a NESTED subsystem `MicrogridACBus/ZigZagGrounding`** tapped in parallel on the bus node (not in the PCC↔VSC series path); its `Out_I_neutral_A` bubbles up to a `MicrogridACBus` outport. Draws ~0 current in normal balanced operation (confirmed).

---

### 3.4.1 `ZigZagGrounding` (NEW v1.4 — CLAUDE.md A9)

**Role:** Zig-zag (interconnected-star) grounding transformer permanently connected to the 33 kV MicrogridACBus. Provides a **breaker-independent ground reference / zero-sequence path** for island mode, since CB1 sits on the PCC Yg side and a three-wire grid-forming VSC supplies no neutral. Prevents the √3≈1.73× healthy-phase overvoltage of an ungrounded system on a single-line-to-ground fault (IEEE Std 142). Optional neutral grounding resistor (NGR) limits ground-fault current.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_AC_33kV` | Bidirectional | Elec (3-φ) | kV / A | ↔ `MicrogridACBus` bus node | Parallel tap on the 33 kV bus (not in series) |
| `Out_I_neutral_A` | Out | Sim | A | scope / protection diagnostic | Neutral / zero-sequence current; ~0 normal, rises on ground fault |

**Internal blocks (AS BUILT — build_mg_acbus.m v1, 2026-06-08):**
- **Zig-zag grounding transformer = native `ee_lib/Passive/Transformers/Earthing Transformer`.** ee_lib SHIPS a dedicated zig-zag earthing/grounding bank — **no zero-sequence-shunt equivalent was needed**. (A heavier `Zigzag-Delta-Wye Transformer` 3-winding block also exists; the Earthing Transformer is the dedicated, simpler grounding device → chosen.) By construction it presents **high +/−-sequence impedance and a low zero-sequence path**. **Verified port map (probed 2026-06-08): `LConn(1)=3-φ ~ terminal`, `RConn(1)=neutral n`.** Params set: `SRated=2 MVA` (% TUNED), `VRated=33e3`, `FRated=50`, `pu_Xl=0.30`, `pu_Rw=0.01` (block defaults). The high magnetizing branch (`pu_Rm=pu_Xm=500`) makes the normal-operation (magnetizing) current negligible.
- **NGR** — single-phase `ee_lib/Passive/Resistor` between the earthing-XFMR neutral and Electrical Reference. `R_ngr=47 Ω` (`% TUNED:` placeholder, ≈400 A SLG-limit @33 kV; size in the fault/island test). Set →~0 for effectively-solid grounding.
- **Neutral current sensor** — single-phase `ee_lib/Sensors & Transducers/Current Sensor` in series in the neutral→NGR→ground path. **Verified port map: `LConn(1)=elec in`, `RConn(2)=elec out`, `RConn(1)=PS signal`.** PS-Simulink converter (unit `A`) → `Out_I_neutral_A`.

**Sign / convention notes:**
- This subsystem **only has an electrical port + ground**; no control inputs (it is passive, always connected).
- It does **not** carry load power; in balanced operation neutral current ≈ 0. **Confirmed in test: I_neutral ≈ 0 A** with CB1 closed and a balanced 300 MW load. Any non-trivial steady current here flags an imbalance/ground-fault condition.

**Resolved / remaining flags:**
- ✓ ee_lib provides a native zig-zag (Earthing Transformer) primitive at R2026a — substitution question closed.
- NGR vs solid grounding (resistance- vs effectively-grounded) remains an open protection-coordination choice — couples to risk **R3** (low inverter fault current already blinds 51G; an NGR lowers ground-fault current further). Coordinate with the protection discussion and size `R_ngr` in the fault test.
- The island-mode ground test (CB1 open → zig-zag is the sole ground reference) is deferred to Task 5.5 (needs the grid-forming VSC to energise the islanded bus).

---

### 3.5 `GridFormingVSC`

**Role:** Averaged 3-φ VSC, V-f droop controller. Interfaces 1500 V DC bus to 33 kV AC bus. **A2 architectural constraint: this is grid-forming → no anti-islanding detection delay.**

**DC-link voltage architecture (decided 2026-05-27):** The VSC **does NOT regulate V_dc** in v1. V_dc regulation is owned by the ESS bidirectional DC-DC converter (see §3.9). VSC is a pure voltage source on the AC side. The VSC has a safety backstop only: if `V_dc < 0.90 · 1500 V = 1350 V` for > 100 ms, VSC reduces AC injection; if `V_dc > 1.10 · 1500 V = 1650 V`, VSC absorbs more from DC. Normally inactive — this is a saturation/SOC-rail backstop only, not a primary regulator. Rationale below.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_AC_33kV` | Bidirectional | Elec (3-φ) | kV / A | ↔ `MicrogridACBus/Elec_VSC_in` | AC side |
| `Elec_DC_1500V` | Bidirectional | Elec (DC, 2-wire) | V / A | ↔ `DCBus/Elec_VSC_term` | DC side |
| `In_V_ref_pu` | In | Sim | pu (base 33 kV) | ← `EMS/Out_V_ref_pu` | Default 1.00 |
| `In_f_ref_Hz` | In | Sim | Hz | ← `EMS/Out_f_ref_Hz` | Default 50.0 |
| `In_mode` | In | Sim | 0/1 (uint8) | ← `EMS/Out_VSC_mode` | 0=grid-following, 1=grid-forming |
| `Out_P_vsc_MW` | Out | Sim | MW | scope / EMS | + = injecting into AC bus |
| `Out_Q_vsc_MVAr` | Out | Sim | MVAr | scope / EMS | |
| `Out_V_dc_V` | Out | Sim | V | scope / DC bus controller | Local measurement (also in DCBus) |

**Internal blocks (AS BUILT — build_vsc.m v1, 2026-06-10):**
- **Average-Value Voltage Source Converter (Three-Phase)** — `ee_lib/Semiconductors & Converters/Converters/...` (block name contains embedded newlines; build_vsc resolves the exact source path by regexp). **Verified port map (probed 2026-06-10): `LConn(1)=PS modulation-wave input (3-vector)`, `LConn(2)=3-φ AC terminal`, `RConn(1)=DC+`, `RConn(2)=DC−`.** **Verified gain law: `v_phase = m · V_dc/2` exactly** (m=0.8, Vdc=1500 → 600.0 V measured). `input_option=ModWave` (default), `FRated=50`, `Vdc_nom=1500`.
- **Internal step-up transformer 0.69/33 kV, 350 MVA (G1)** — with V_dc=1500 V the gain law caps the AC side at 750 V phase-peak, so 33 kV is physically unreachable without a transformer. Converter LV = **690 V LL** (m̂=0.75 at 1.0 pu → 33% headroom), then Two-Winding Transformer 350 MVA, 10% leakage (= the **coupling impedance** — two voltage sources must never parallel without impedance). **Vector group LV=Yg / MV=Δ11 (G2):** the Δ MV side is 3-wire → the VSC contributes NO ground reference, so the **A9 zig-zag stays the sole island-mode ground** (a Yg here would silently add a second ground source).
- **MATLAB Function `vf_droop`** (one block, discrete 50 µs, persistent states): Clarke P/Q calc + LPF (τ=10 ms, G5) → primary droop → A8 secondary PIs → θ oscillator → `m_abc` modulation wave. Droop params `R_p = R_q = 0.05`, base = 350 MVA (% TUNED).
  - **⚠ Droop notation fix (G4):** the v1.1 equation here read `f_cmd = f_ref − (1/R_p)·P_pu·f0`. Read literally with R_p=0.05 that is a gain of 20 → 1000 Hz swing at rated P — unphysical. **As built:** the standard **slope** form `f_cmd = f_ref_eff − R_p·(P/S_base)·f0` (5% droop: rated P → 2.5 Hz) and `V_cmd = V_ref_eff − R_q·(Q/S_base)`. The "1/R" form belongs to the inverse (governor-response) direction.
  - **Soft-start (G3) + pre-sync sync breaker (G8, added 2026-06-20):** amplitude reaches 100% of the droop target over 200 ms, BUT on an already-energized bus a from-zero ramp is a near-short — the converter is a ~0 V source behind 10% leakage. A native `Circuit Breaker (Three-Phase)` **`CB_sync`** sits between the step-up MV winding and the bus port (same inverted-control semantics as PCC §3.3: ctrl>0.5=OPEN). It starts **OPEN** (so the t=0 m=0 instant only magnetizes the unloaded transformer — no fault IC), the bus-side V/I sensors pre-synchronize the LV oscillator to (φ_bus − 30°, the Yg/Δ11 +30° shift) and ramp amplitude to the measured |V|, then **CLOSE** when `soft=1 ∧ |Δθ|<0.03 rad`; amplitude blends measured→droop over 100 ms; secondary integrators unfreeze after the blend. Dead bus (|V|<0.05 pu, island black start): close immediately and energize via the plain ramp. **The `cb_S2PS` converter must use `provide` (no input filter) — a filter state starts at 0 < 0.5 = CLOSED, re-creating the very fault the breaker prevents.** Task 5.5 never saw this because the TEST-ONLY ideal DC source bankrolled the soft-start short invisibly.
  - **V_dc-adaptive modulation (G6):** `m = V_target_pk/(V_dc_meas/2)` (guard ≥750 V), saturated at m=1.0. This linearises against DC sag AND is the natural §3.5 saturation backstop (V_dc collapse → m ceiling → AC sags; no energy from nothing). The explicit 100 ms-timer backstop logic is deferred to EMS/Task 6.
  - **Algebraic-loop break:** `m_abc` is direct feedthrough from same-step V/I measurements → the loop sensors→droop→m→network→sensors must be broken with a **Unit Delay (1 sample, 50 µs)** before the Simulink-PS converter. A 1st-order S-PS input filter does NOT break it (the filter derivative x′=(u−x)/τ still depends directly on u); the τ=1e-4 filter is kept only to give the DAE a smooth derivative (1.8° at 50 Hz, absorbed by the droop equilibrium). One sample is also the honest model of controller computation delay.
- Mode logic: grid-following (mode=0) is a **stub (G7)** — f_cmd tracks `f_meas`, droop/secondary frozen, V_cmd=V_ref. The project runs grid-forming (mode=1, A2) throughout.
- MV measurements: 3-φ Current Sensor (series, +I = VSC→bus) + Phase Voltage Sensor at the 33 kV boundary feed the droop and Out_P/Q. DC: 1-φ Voltage Sensor (∥) → Out_V_dc_V.

**Note:** no dq-frame inner current loop (deferred per the original §3.5 note) — modulation index driven directly by the droop output. Adequate for f/control/EMS at the averaged-model scope (R5); add the inner loop only if HIL exposes the need.

**Secondary control (A8 — AS BUILT in vf_droop):** two slow PIs (`Kp_f=0.1, Ki_f=0.5 /s`, clamp ±0.5 Hz; `Kp_V=0.5, Ki_V=2.0 /s`, clamp ±0.1 pu — all % TUNED) trim the refs: `f_ref_eff = f_ref + PI_f(f_ref − f_meas)`, `V_ref_eff = V_ref + PI_V(V_ref − V_pcc)`. Anti-windup per Rule 7: conditional integration (frozen while the modulator saturated last step or during soft-start) + hard clamps. Inputs are the global Froms `f_meas`/`V_pcc`. **Enable/setpoints pass to EMS in Task 6** (currently always on). Hierarchy: primary droop (ms) → secondary restoration (s) → tertiary/EMS (A6); Rocabert 2012, Guerrero.

**Validated (build_vsc_test.m, 2026-06-10, Transmission+PCC+MGbus+VSC+DCBus, CB1 CLOSED, 300 MW @pf0.95, TEST-ONLY 1500 V DC source, ode14x@50µs, 4 s):**
**V_pcc = 1.0003 pu** (recovered from 0.945 without VSC — the A8 V-restoration works), f_meas = 50.018 Hz stable, V_dc = 1499.5 V stable, **Q_vsc = +98.1 MVAr** (VSC supplies ≈ the entire load Q locally; PCC then imports ≈pure P), P_vsc = +0.64 MW (small f-droop/secondary pickup), P_pcc = +299.5 MW. Power balance closes: P_pcc + P_vsc ≈ 300.2 MW = load + losses. Transmission cap-IC relax warning = known pre-existing quirk (non-fatal). Caveat: transmission SMs ran with D_pu=30 (settling aid) → f dynamics over-damped vs Layer-1; Task 5.2b restores D=1.5. **Island-mode test (CB1 open) deliberately NOT run — separate careful session.**

---

### 3.6 `DCBus`

**Role:** 1500 V common DC node. **Passive electrical node**; voltage is regulated externally by ESS DC-DC (primary) with VSC saturation backstop (see §3.5).

**Capacitor sizing (v1):** `C_dc` is sized so that for a worst-case ±50 MW step on DC side with ESS PI response time τ_pi = 50 ms (single-step lag of inner DC-DC loop), V_dc stays within ±5%. From `C · V · dV/dt ≈ ΔP`: `C_dc ≥ ΔP·τ / (V·ΔV) = 50e6 · 0.05 / (1500 · 75) ≈ 22 mF`. Use **30 mF** as starting value, tune in island test.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_VSC_term` | Bidirectional | Elec (DC) | V / A | ↔ `GridFormingVSC/Elec_DC_1500V` | |
| `Elec_PV_term` | Bidirectional | Elec (DC) | V / A | ↔ `PV/Elec_DC_out` | PV side |
| `Elec_Wind_term` | Bidirectional | Elec (DC) | V / A | ↔ `Wind/Elec_DC_out` | Wind side |
| `Elec_ESS_term` | Bidirectional | Elec (DC) | V / A | ↔ `ESS/Elec_DC_io` | Battery side |
| `Elec_Elec_term` | Bidirectional | Elec (DC) | V / A | ↔ `Electrolyzer/Elec_DC_in` | Electrolyzer side |
| `Elec_FC_term` | Bidirectional | Elec (DC) | V / A | ↔ `PEMFC/Elec_DC_out` | FC side |
| `Out_V_dc_V` | Out | Sim | V | scope / EMS | Voltage Sensor on bus |
| `Out_V_dc_pu` | Out | Sim | pu (base 1500 V) | scope (optional) | |

**Internal blocks (AS BUILT — build_dcbus.m v1, 2026-06-10):**
- **Passive star node:** all six `Elec_*_term` PMIOPorts tie to ONE internal node (the + rail). **Grounded-negative-rail convention (F1):** a Simscape conserving port is a single node, so the "2-wire" DC bus is implemented as + rail through the ports, − rail = Electrical Reference inside every DC subsystem (electrically identical; realistic for functionally-grounded 1500 V DC). The VSC's internal Yg/Δ transformer galvanically isolates this DC ground from the 33 kV AC grounding (A9 zig-zag unaffected).
- **Capacitor** `ee_lib/Passive/Capacitor` (LConn/RConn = the two terminals): `c = 30 mF` (% TUNED — §3.6 sizing rule; revisit in the island test, R2), ESR `r = 1 mΩ` (mild R2 damping), **initial condition `vc = 1500 V` priority High (F2)** — without it the test's stiff source dumps a meaningless inrush at t=0.
- **Voltage Sensor** (1-φ, verified map `LConn1=+, RConn1=PS out, RConn2=−`) ∥ on the bus → `Out_V_dc_V` and ×(1/1500) → `Out_V_dc_pu`.
- 5 of 6 terminals stay open at top level until the source modules exist (Tasks 5.6+); in tests the **TEST-ONLY ideal 1500 V source (1 mΩ)** hangs on `Elec_ESS_term` from the harness script only — never from build_dcbus.m, never saved.

**Validated (build_vsc_test.m, 2026-06-10):** V_dc = 1499.5 V stable under VSC load (see §3.5 validation block).

---

### 3.7 `PV`

**Role:** physical single-diode PV array + Perturb&Observe MPPT + averaged boost DC-DC. Feeds the 1500 V DC bus.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_DC_out` | Bidirectional | Elec (DC) | V / A | ↔ `DCBus/Elec_PV_term` (LConn2) | one CCS, I = P_bus/V_dc |
| `In_irr_Wm2` | In | Sim | W/m² | ← From Workspace `pv_data.G_ts` (full model) / const (test) | plane-of-array (= direct+diffuse)·1000 |
| `In_T_amb_C` | In | Sim | °C | ← From Workspace `pv_data.T_ts` / const | ambient (cell temp via NOCT inside) |
| `Out_P_pv_MW` | Out (1) | Sim | MW | scope / EMS | DELIVERED to bus (= P_panel·η_dcdc·η_sys) |
| `Out_V_pv_V` | Out (2) | Sim | V | scope | array operating voltage (MPPT V_op) |
| `Out_I_pv_A` | Out (3) | Sim | A | scope | array current at V_op |
| `Out_irr_Wm2` | Out (4) | Sim | W/m² | scope | pass-through of In_irr_Wm2 |

**AS-BUILT (build_pv.m v1, validated 2026-06-20) — HYBRID PHYSICAL model (Task 5.7).** Supersedes the v1.1 "controllable DC source from `real_profiles.P_pv`" plan: the RAW irradiance + temperature drive a physical engine; Renewables.ninja's processed `electricity` is the VALIDATION reference only, NOT an input. Params in `DATA/pv_params.mat` (`build_pv_params.m`, Rule 8); diode physics in `PARAMS/pv_iv.m` (offline twin) + `PARAMS/pv_mpp.m` (offline MPP sweep).
- **P1 — signal-level (ESS H1 pattern).** Diode I–V, MPPT V_ref, and boost are MATLAB Functions; the whole electrical PV footprint on the bus = ONE `Controlled Current Source`, I = P_bus·1e6/V_dc (RConn1=PS cmd, LConn1=+rail, RConn2=gnd). No physical panel node at averaged scope (R5).
- **`pv_cell`** (single-diode, params injected as literals = inline twin of `pv_iv.m`): `I = I_ph − I_0(exp(V/a)−1)`, `I_ph∝G·(1+α(T_cell−25))`, `a = N_s·n·k·T_K/q`, `V_oc(T)` via `β=−0.4%/°C`, `T_cell = T_amb + (NOCT−20)/800·G` (NOCT=45). One equivalent array (NOT cell-by-cell): `V_oc_stc=900 V`, `N_s=1500`, `n=1.3`, `I_sc_stc` scaled so STC MPP = 80 MW (P_mpp ∝ I_sc exactly).
- **`mppt_po`** (Perturb & Observe): perturbs V_ref every 1 ms (period/dt=20), ΔV=2 V, clamp [50, 900] V. Converges to the swept MPP (validated: V_op matches `pv_mpp` to <1 V at 4 conditions).
- **`dcdc_ctrl`** (averaged boost): `P_bus = P_panel·η_dcdc(0.98)·η_sys(0.80)`, `I_cmd = P_bus/max(V_dc,750)`. η_sys = performance-ratio derate (non-thermal; temperature is in the diode) — calibrated to land annual energy within 6% of ninja.
- **Two Unit Delays** (ESS H6 lesson): V_ref→V_op breaks pv_cell↔mppt; I_cmd→CCS breaks V_dc↔I=P/V_dc↔network.
- **P2 — Inports, not internal From Workspace**: In_irr/In_T_amb are driven externally (test=constants, full model=From Workspace `pv_data` at master level, as the load profiles are). Keeps the module isolated-testable.

**Validated (build_pv_test.m, 2026-06-20; isolated PV+DCBus, TEST-ONLY 1500 V source, ode14x@50 µs):**
- (A) Simulink MPPT at 4 conditions — P&O lands on the offline swept MPP exactly: STC 62.7 MW @646 V, summer-hot(32 °C) 60.2 MW, winter-clear(5 °C,900 W/m²) 63.9 MW, cloudy(300 W/m²) 21.9 MW. Hotter → less power (NOCT+β physics).
- (B) Offline physics vs ninja over 8760 h: annual **131.8 vs 124.8 GWh (ratio 1.056, within ±10 %)**, daylight shape **corr 0.9999**, peak 71.1 vs 68.2 MW; temp derating G=900 10→40 °C: 79.5→67.1 MW (−15.6 %). A full year at 50 µs (~1.7e9 steps) is infeasible → the physics is validated offline, the MPPT at sample points.
- ⚠ Data note: the ninja CSV (`capacity=1`) is NORMALIZED — `electricity` is per-unit (×80 MW for the plant), `irradiance` is kW/m² (×1000 → W/m²); year is **2025** (CLAUDE.md said 2023).

---

### 3.8 `Wind`

**Role:** physical Cp(λ,β) + four-region power-curve turbine + averaged generator/rectifier. Feeds the 1500 V DC bus.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_DC_out` | Bidirectional | Elec (DC) | V / A | ↔ `DCBus/Elec_Wind_term` (LConn3) | one CCS, I = P_bus/V_dc |
| `In_v_wind_ms` | In | Sim | m/s | ← From Workspace `wind_data.v_ts` (full model) / const (test) | RAW wind speed |
| `Out_P_wind_MW` | Out (1) | Sim | MW | scope / EMS | DELIVERED to bus (= P_turb·η_gen·η_wind) |
| `Out_v_wind_ms` | Out (2) | Sim | m/s | scope | pass-through |
| `Out_Cp` | Out (3) | Sim | – | scope | Cp_max in cube region, falls in pitch region |

**AS-BUILT (build_wind.m v1, validated 2026-06-20) — HYBRID PHYSICAL model (Task 5.8), mirrors PV §3.7.** RAW wind speed drives the physics; Renewables.ninja `electricity` (a Vestas V90 2000 curve) is the VALIDATION reference only. Params `DATA/wind_params.mat` (`build_wind_params.m`, Rule 8); `PARAMS/cp_lambda.m` = the analytic Cp(λ,β) (Heier) used offline to find Cp_max.
- **W1 — signal-level (PV P1)**: turbine + generator are MATLAB Functions; the whole electrical footprint = ONE `Controlled Current Source`, I = P_bus·1e6/V_dc.
- **`wind_turbine(v)`** (four-region, params as literals): below `cut_in`(3 m/s) or ≥`cut_out`(25) → P=0; cube region runs at **Cp_max=0.48** (= cp_lambda at λ_opt=8.1, β=0 — the wind analogue of MPP), `P = Cp_max·½ρAv³`; above `v_rated`(10) the turbine pitches → `P = P_rated`(120 MW), effective `Cp = P_rated/(½ρAv³)` < Cp_max. Aggregate rotor area `A = P_rated/(Cp_max·½ρ·v_rated³)` (one equivalent turbine, R_eq≈360 m).
- **W2 — quasi-static optimal-λ** (no rotor-inertia / pitch-controller dynamics; adequate at R5). → only ONE algebraic-loop Unit Delay (DC-side I=P/V_dc); `wind_turbine(v)` is feed-forward (PV has a second loop for its MPPT).
- **`dcdc_ctrl`**: `P_bus = P_turb·η_gen(0.95)·η_wind(0.90)`, `I = P_bus/max(V_dc,750)`. η_wind = wake+availability derate (% TUNED; an aggregate farm, unlike ninja's single turbine).
- **W3 — Inport In_v_wind_ms** (PV P2): test=const, full model=From Workspace `wind_data`.

**Validated (build_wind_test.m, 2026-06-20; isolated Wind+DCBus, TEST-ONLY 1500 V source, ode14x@50 µs):**
- (A) Simulink four-region — all correct: v=2 → P=0 (cut-in); v=8 → 52.5 MW @Cp=0.48 (cube); v=15 → 102.6 MW @Cp=0.14 (rated plateau, pitched); v=28 → P=0 (cut-out). Matches offline exactly.
- (B) Offline vs ninja (8760 h): annual **320.1 vs 333.6 GWh (ratio 0.959, within ±10 %)**, shape **corr 0.9913**, CF 0.305 vs 0.317; the power-vs-windspeed bins track ninja's V90 in the cube region (v=5: 0.11/0.13, v=8: 0.44/0.48 pu). Peak 102.6 vs 118.9 MW — ours is lower by the wake/availability derate (an aggregate farm). v_rated=10 m/s tuned to best fit ninja's profile.
- ⚠ Data note: ninja CSV (`capacity=1`) NORMALIZED — `electricity` per-unit (×120 MW), `wind_speed` m/s; year **2025**; turbine ref Vestas V90 2000 (ours is a generic Cp curve → curves needn't match exactly, only the profile + four regions).

---

### 3.9 `ESS` (Battery + Bidirectional DC-DC) — **owns DC-bus voltage regulation**

**Role:** 80 MW / 200 MWh battery storage. **Primary DC bus voltage regulator** (see §3.5 rationale). Tracks SOC. Accepts EMS power reference as feed-forward; V_dc PI provides the fast trim.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_DC_io` | Bidirectional | Elec (DC) | V / A | ↔ `DCBus/Elec_ESS_term` | |
| `In_Pref_MW` | In | Sim | MW | ← `EMS/Out_Pess_ref_MW` | EMS dispatch target. Feed-forward only. Range [-80,+80]; + = discharge |
| `Out_SOC_ess` | Out (1) + Goto | Sim | 0–1 | → `SOC_ESS` (global) | Bounded [0.15, 0.90] (Rule 7) |
| `Out_P_ess_MW` | Out (2) | Sim | MW | scope / EMS | Realised P after slew + V_dc trim + clamp |
| `Out_V_batt_V` | Out (3) | Sim | V | scope | Thevenin diagnostic OCV(SOC)−R·I |

**AS-BUILT (build_ess.m v1, validated 2026-06-20).** Differs from the v1 plan above in several places — design decisions H1–H7 in the build_ess.m header:
- **H2 — self-measured V_dc.** No `In_Vdc_meas_V`/`In_Vdc_set_V` ports: the ESS senses V_dc with its OWN parallel `Voltage Sensor` at `Elec_DC_io` (same star node as DCBus) and holds the 1500 V setpoint internally. Self-contained.
- **H1 — signal-level Thevenin battery.** The battery (OCV(SOC)=700+SOC·200 V, R_int=1.6e-4 Ω, % TUNED) is computed INSIDE the `ess_ctrl` MATLAB Function, not built as a 2nd electrical network (which would need its own Solver Configuration and buy nothing at averaged scope, R5). **The entire electrical ESS = ONE `Controlled Current Source` on the + rail** (same pattern as PV/Wind §3.7/§3.8). CCS verified port map: `RConn(1)`=PS command [A]; `LConn(1)` & `RConn(2)`=terminals; **+command injects OUT of LConn(1)** (probed: +100 A into 10 Ω → +1000 V) → LConn1 = + rail, RConn2 = ground (F1).
- **H6 — Unit Delay** (1 sample) breaks the sensor→ctrl→CCS→network algebraic loop; the filtered S-PS converter (τ=1e-4) supplies the DAE derivative (same lesson as the VSC).

**Control law (`ess_ctrl`, one 50 µs discrete MATLAB Function):**
```
err_V      = V_dc_set - V_dc_meas                                [V_dc_set = 1500]
p_int      = clamp(p_int + Ki·err_V·dt, ±Pmax)  if not saturated [Ki = 5 MW/V/s; Rule 7 anti-windup]
pref_slew  = rate_limit(Pref_MW, slew = 80 MW/s)                 [H7: slew the EMS FF ONLY]
P_sum      = pref_slew + Kp·err_V + p_int                        [Kp = 0.2 MW/V  ← % TUNED, NOT 0.5]
P_actual   = clamp(P_sum, -Pmax(SOC), +Pmax(SOC))               [Rule 4 single TOTAL clamp, SOC-aware]
I_cmd_A    = P_actual·1e6 / max(V_dc_meas, 750)                  [bus-side CCS command]
SOC       -= P_batt·dt/(3600·200);  P_batt = P_act/η (dis) | P_act·η (chg)   [η = 0.95, H5]
```
- **Kp = 0.2 MW/V (NOT the §3.9-plan 0.5).** H3: crossover ωc≈Kp/(C_dc·V0)≈4.4 krad/s — ×70 above the droop power poles, ×14 below Nyquist; 0.5 rings at 50 µs sampling. R2 margin: VSC CPL |Z|=V²/P≥28 mΩ vs ESS output-Z≈V0/Kp≈7.5 mΩ → ~3.7×.
- **H7 — slew (80 MW/s) on the EMS feed-forward ONLY, NOT on the PI trim.** Slewing the summed command starved the fast V_dc loop (80 MW/s = 4 kW per 50 µs) → limit cycle → divergence at 6–8 ms. The averaged DC-DC has no sub-ms power slew; Rule 4's total clamp still bounds |P|≤80 MW.

**Anti-windup:** integrator frozen while the previous step's TOTAL clamp engaged (Rule 7).

**Validated (build_ess_test.m, 2026-06-20; grid-connected CB1 CLOSED, 3 s, ode14x@50 µs).** ESS is the SOLE V_dc source (the Task 5.5 TEST-ONLY ideal 1500 V source is REMOVED). Steady: V_dc=1497.6 V, V_pcc=0.994 pu, f=49.995 Hz, SOC=0.6000 (flat), Q_vsc=82 MVAr. Load step 300→320 MW (rate-limited ~50 ms) at t=2 s: V_dc held, transient stayed **1467.5–1526.0 V — well inside ±5 % [1425,1575]** (R2 ✓), no ringing (residual ±2 V); post-step V_dc=1500.2 V, P_pcc 297.6→319.3 MW (grid absorbs the step, ESS only trims V_dc). Balance closes (constant-Z load draws P·(V/Vr)²).
- ⚠ **Two test-harness prerequisites, not ESS bugs** (see CLAUDE.md status 2026-06-20): (1) the transmission network's lightly-damped ~2–3 Hz electromechanical common mode rings ±25 MW at t=2 s under D_pu=30 and the load step tips it into divergence → **D_pu raised 30→60** in the test (settling knob; Task 5.2b restores →1.5). It was masked in Task 5.5 by the stiff ideal source + window-averaged report. (2) the instantaneous 50 µs load step is an unphysical near-impulse → **rate-limited to ~50 ms**.

**Why ESS not VSC owns V_dc (rationale, kept here for design reviewers):**
1. **Architectural separation of concerns.** With ESS-owned V_dc, the VSC stays a clean voltage source on the AC side (the textbook grid-forming archetype, e.g. Rocabert 2012, Tayyebi 2020). VSC's job is "be the grid"; ESS's job is "balance the DC node."
2. **Fast power buffer is the battery, not the AC grid.** ESS has bidirectional DC-DC with ~50 ms response and a large MWh buffer — exactly what V_dc regulation needs. Routing DC-side transients through the VSC to the AC grid (and through the synchronous machines beyond) adds chain delays.
3. **Less cross-coupling between AC-frequency droop and DC-voltage droop.** VSC owning both V_dc and f means a DC-side disturbance perturbs f and vice versa. ESS-owned V_dc keeps the two loops orthogonal.
4. **Drag-and-drop friendliness.** Future team members reading the model see ESS = "battery box that holds DC voltage steady" and VSC = "AC-side voltage source". Two simple stories beat one tangled one.

**Known limit (documented for honesty):** When SOC rails (SOC ≈ SOC_min and net DC load, or SOC ≈ SOC_max and net DC injection), ESS cannot regulate. VSC's saturation backstop (§3.5) takes over by limiting AC injection/absorption. If both rail, EMS must curtail (PV/Wind curtailment or load shedding) — that is an EMS Task-6 responsibility, not a v1 model concern.

---

### 3.10 `Electrolyzer`

**Role:** physical PEM electrolyzer (Ulleberg polarization + averaged buck + Faraday H2) — a LOAD on the 1500 V DC bus.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_DC_in` | Bidirectional | Elec (DC) | V / A | ↔ `DCBus/Elec_Elec_term` (LConn5) | Consuming; one CCS, I = −P/V_dc |
| `In_Pref_MW` | In | Sim | MW | ← `EMS/Out_Pelec_ref_MW` | 0 (off) or [5, 40]; min nonzero to avoid degradation |
| `Out_P_elec_MW` | Out (1) | Sim | MW | scope / EMS | drawn power, after slew + τ_E = 30 s lag |
| `Out_mdot_H2_kgs` | Out (2) | Sim | kg/s | → `H2Tank/In_mdot_prod_kgs` (WIRED 2026-06-25, master + chain test) | Faraday (see below); + = production |
| `Out_U_stack_V` | Out (3) | Sim | V | scope | aggregate stack voltage (rises with power) |

**AS-BUILT (build_electrolyzer.m v1, validated 2026-06-20) — HYBRID PHYSICAL (Task 5.9).** Supersedes the v1 "CCS + flat H2 gain" sketch. Shared scalars from `h2_params.mat` struct `elec` (Rule 8: n_cells=400, T=80 °C, U_rev=1.229, F, P_rated=40, τ=30 s); Ulleberg coefficients + aggregate area + efficiencies as literals.
- **E1 — Ulleberg (2003) polarization** (`electrolyzer_cell`, a LOCAL function inside `el_ctrl` so the P→I solve can call it):
  `U_cell = U_rev + (r1+r2·T)·i + s·log10((t1+t2/T+t3/T²)·i + 1)`, i=I/A [A/m²], T=80 °C fixed (R5).
  Terms: U_rev=thermodynamic threshold; ohmic (membrane/electrode iR); activation (Tafel). Coeffs r1=8.05e-5, r2=−2.5e-7, s=0.185, t1=−0.1002, t2=8.424, t3=247.3 (Ulleberg); aggregate area **A=15 m²** (% TUNED) + N_cells=400 land the curve at ~70 % over 5–40 MW.
- **E2 — P→I inversion by bisection** (40 iters; P(I)=N·U_cell(I)·I monotone, I ≤ P/(N·U_rev)). Couples H2 to the actual stack point (U_stack genuinely rises with power, not decorative).
- **E3 — Faraday (current-based, the physical law):** `ṁ_H2 = η_F·N_cells·I_stack/(2F)·M_H2` (η_F=0.99, 2 e⁻/H2). With the curve sized to ~70 % this EMERGES as ≈21 kg/MWh (LHV) — ⚠ the v1 note `= P·21/3600·η_E` **double-counted η** (the 21 already includes η=0.70: 1000 kWh·0.70/33.3=21). Faraday-on-current avoids that.
- **E4 — τ=30 s is BALANCE-OF-PLANT** (pump/thermal/separation), not the sub-second electrochemistry (R5): a slewed setpoint (independent state) + first-order lag on the realized power.
- **E5 — LOAD-direction CCS** (build_ess.m H1 pattern): I_cmd = −P/V_dc draws from the +rail; Unit Delay breaks the V_dc→I→CCS→network loop.
- **E6 — Rule 4** single total clamp (0 or [5,40] MW); no PI → no windup (Rule 7 trivial).

**Validated (build_electrolyzer_test.m, 2026-06-20; isolated EL+DCBus, TEST-ONLY 1500 V source, 120 s, ode14x@50 µs):** command 0→20(t=10)→40(t=60) MW. τ check: P_elec reaches 63 % at t=40.6 s → **τ≈30.6 s** ✓. Polarization: U_stack 492 (off=N·U_rev) → 664 → 729 V (rises with power) ✓. H2: at realized 15.78 MW → 343 kg/h = **21.74 kg/MWh (72 % LHV)**; at 36.41 MW → 19.82 kg/MWh (66 %) — straddles the 21.02 of `h2_params.elec.h2_rate` ✓. Buck draws correctly (V_dc dips via the TEST source's 1 mΩ).

---

### 3.11 `PEMFC`

**Role:** physical PEM fuel cell (Correa polarization + averaged boost + Faraday H2 consumption) — a SOURCE on the 1500 V DC bus. The source-direction twin of §3.10.

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `Elec_DC_out` | Bidirectional | Elec (DC) | V / A | ↔ `DCBus/Elec_FC_term` (LConn6) | Sourcing; one CCS, I = +P/V_dc |
| `In_Pref_MW` | In | Sim | MW | ← `EMS/Out_Pfc_ref_MW` | 0 (off) or [2, 30]; never negative |
| `Out_P_fc_MW` | Out (1) | Sim | MW | scope / EMS | injected power, after slew + τ_FC = 5 s lag |
| `Out_mdot_H2_cons_kgs` | Out (2) | Sim | kg/s | → `H2Tank/In_mdot_cons_kgs` (WIRED 2026-06-25, master + chain test) | Faraday consumption; + = removes tank mass |
| `Out_V_stack_V` | Out (3) | Sim | V | scope | aggregate stack voltage (DROPS with power — FC) |

**AS-BUILT (build_fc.m v1, validated 2026-06-20) — HYBRID PHYSICAL (Task 5.10), source-twin of the electrolyzer §3.10.** Shared scalars from `h2_params.mat` struct `fc` (Rule 8: n_cells=600, A_cm2=500, T=75 °C, P_rated=30, τ=5 s, i_L=1.5=J_max); Correa coefficients + aggregate N_p as literals.
- **vs the electrolyzer twin:** SOURCE (I_cmd=+P/V_dc, injects; EL drew −); BOOST (low stack V→1500 V); H2 CONSUMED (negative tank input); τ=5 s (EL 30 s); V_cell DROPS with current (EL's U rose); P(J) NON-monotonic (peaks ≈75 MW) → invert on the rising branch only.
- **C1 — Correa (2004) polarization** (`fc_cell`, LOCAL fn): `V_cell = E_nernst − V_act − V_ohm − V_con` — V_act (activation, low-i Tafel ξ1..ξ4), V_ohm (i·(R_M+R_C)), V_con (−B·ln(1−J/J_max), gas starvation near J_max). T=75 °C fixed (R5); p_H2=1.5, p_O2=1.0 atm. ξ1=−0.948, ξ3=7.6e-5, ξ4=−1.93e-4 (ξ2 from A, C_H2), B=0.016, R_M+R_C=2.4e-4 Ω (= h2_params R_int). Aggregate N_p=408 (% TUNED) → rated 30 MW at V_cell≈0.70, ~55 %.
- **C2 — P→J bisection** on J∈[0,1.0] (monotone below J_peak≈1.3); P_stack=K·V_cell(J)·J, K=N_s·N_p·A_cell.
- **C3 — Faraday CONSUMPTION:** `ṁ = (P_stack/V_cell)·M_H2/(2F·η_util)` (η_util=0.98). ⚠ the v1 note `= P·54.6/3600/η_FC` **double-divided** η (54.6 already = 1000/(33.3·0.55)); current-based Faraday avoids it.
- **C4 — τ=5 s BoP** (air compressor + manifold/O₂), independent slewed setpoint + lag (the EL lag-bug lesson). **C5 — SOURCE CCS** I=+P/V_dc + Unit-Delay loop break. **C6 — Rule 4** clamp (0 or [2,30]).

**Validated (build_fc_test.m, 2026-06-20; isolated FC+DCBus, TEST-ONLY 1500 V source, 50 s, ode14x@50 µs):** command 0→15(t=5)→30(t=25) MW. τ check: P_fc 63 % at t=10.6 s → **τ≈5.6 s** ✓ (faster than EL's 30 s). Polarization: V_stack 686 (off=OCV) → 470 → **421 V — DROPS with power** ✓ (opposite EL). Consumption: realized 14.46 MW → 50.5 kg/MWh (60 %); 29.80 MW → 56.4 kg/MWh (53 %) — straddles `h2_params.fc.h2_consume` 54.6 ✓. Boost SOURCES: V_dc rises 1500→1520 (FC injects; EL's bus dropped) ✓.

---

### 3.12 `H2Tank`

**Role:** Signal-level (non-electrical) tank model. 1000 kg at 350 bar. **NO electrical content** — pure Simulink mass-balance integrator; the H2 chemistry is decoupled from the 50 µs electrical domain (averaged-model scope, R5).

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| `In_mdot_prod_kgs` | In | Sim | kg/s | ← `Electrolyzer/Out_mdot_H2_kgs` | + adds mass (production) |
| `In_mdot_cons_kgs` | In | Sim | kg/s | ← `PEMFC/Out_mdot_H2_cons_kgs` | + removes mass (consumption; subtracted internally) |
| `Out_m_H2_kg` | Out | Sim | kg | scope | Current mass in tank, bounded [50, 950] |
| `Out_SOC_H2` | Out (1) + Goto | Sim | 0–1 | → Goto `SOC_H2` (global) | = m/1000, bounded [0.05, 0.95] |
| `Out_P_tank_bar` | Out | Sim | bar | scope | Z-corrected real-gas: P = Z·m·R·T/(M_H2·V_tank) |

**AS-BUILT (build_h2tank.m v1, validated 2026-06-25) — the LAST organ module (11/12), Task 5.11.** Shared scalars from `h2_params.mat` struct `tank` (Rule 8: capacity_kg=1000, P_max_bar=350, T_C=25, M_H2=0.002016, R_gas=8.314); Z-factor + SOC bounds + IC as literal decisions.
- **T1 — signal-level only.** No Solver Configuration / Electrical Reference / Simscape / CCS / PS converters. Just Inports → one MATLAB Function → Outports + Goto. The single signal-level module in the model.
- **T2 — mass balance** = forward-Euler discrete integrator (`tank_dynamics`, persistent `m`): `m(k+1)=m(k)+(mdot_prod − mdot_cons)·dt`. Both inputs arrive **+kg/s** (§1.1) — consumption is subtracted. dt-exact → rate-agnostic; runs at the model FixedStep (50 µs full model; coarse in the standalone test) like the EL/FC functions (§5.1's nominal 100 ms H2 rate is equivalent — re-rate later if desired, avoids rate-transition blocks).
- **T3 — SOC bounds [0.05, 0.95] → m ∈ [50, 950] kg** (Rule 7). The clamp **is** the anti-windup freeze: at a rail the state HOLDS (a full tank cannot overfill, an empty tank cannot go negative — a genuine physical constraint, not just a guard). The EMS also manages these rails at the system level (R1, Task 6), but the tank self-protects. **IC `m0 = 500 kg`** (half-full, per the task brief). ⚠ `h2_params.tank.SOC_init = 0.30` (→300 kg) — reconcile in `hydrogen_system_params.m` if 0.30 is the intended IC.
- **T4 — Z-corrected real-gas pressure** (NOT plain ideal gas): `P = Z·m·R·T/(M_H2·V_tank)`, **Z = 1.2** (H2 compressibility @350 bar — small molecule / repulsive-co-volume regime; real P is ~20 % above ideal for the same mass, so ideal gas UNDER-reads pressure / OVER-reads stored mass by ~20 %). **V_tank is DERIVED** so a full tank reads exactly 350 bar: `V_tank = Z·m_cap·R·T/(M_H2·P_max) ≈ 42.16 m³`. ⚠ This **supersedes `h2_params.tank.V_tank_m3 = 10`**, which is unphysical (1000 kg/10 m³ = 100 kg/m³ vs real gaseous H2 ≈ 23.7 kg/m³ at 350 bar/25 °C, NIST; ideal gas at V=10 m³ would read ~1230 bar at full). The derived 42.16 m³ → 23.7 kg/m³, NIST-consistent. **Flag to update hydrogen_system_params.m** (V_tank_m3 10→42.2, add Z).
- **T5 — Goto `SOC_H2`** (global, TagVisibility=global): H2Tank is the §2 owner of this tag (mirrors ESS owning `SOC_ESS`). EMS (Task 6) + scopes consume via From.

**Validated (build_h2tank_test.m, 2026-06-25; isolated, FAKE mdot constants, FixedStepDiscrete @0.1 s — no Simscape, no grid, no EL/FC):**
- **A (production only, 0.1 kg/s):** fills 500→950 kg (SOC 0.50→0.95), P 175→332.5 bar, then **FREEZES at 950 kg** (max m = 950.00, anti-windup ✓).
- **B (consumption only, 0.1 kg/s):** empties 500→50 kg (SOC 0.50→0.05), P 175→17.5 bar, then **FREEZES at 50 kg** (min m = 50.00 ✓).
- **C (balanced, 0.1 = 0.1):** mass flat at 500 kg (net-zero ✓).
- **Pressure model:** linear in mass, ratio P_real/P_ideal = 1.200 exactly at all masses; **full tank (1000 kg) = 350.0 bar** ✓ (target 350); ideal under-reads by 17 %. Sim P matches the offline Z formula (500 kg→175.0, 950 kg→332.5 bar).

**Hydrogen chain WIRED & validated (2026-06-25, build_h2chain_test.m):** `Electrolyzer/Out_mdot_H2_kgs → In_mdot_prod_kgs` and `PEMFC/Out_mdot_H2_cons_kgs → In_mdot_cons_kgs` are now connected (direct port-to-port lines, NOT Goto/From) at the master level (`build_microgrid_eelib.m` step [6b/7]) and exercised by the closed-loop test (DCBus + EL + FC + H2Tank + TEST-ONLY 1500 V source, NO grid, 120 s, ode14x@50 µs). Three phases — EL 20 MW/FC 0 → **fills +2.17 kg**; EL 0/FC 15 MW → **empties −5.04 kg**; EL 20/FC 15 → **net empties −5.77 kg** (FC 54.6 kg/MWh consumes faster than EL 21 kg/MWh produces at comparable MW). SOC_H2 stayed in [0.4914, 0.5023] ⊂ [0.05, 0.95]; Faraday ratios EL 22.0 / FC 50.7 kg/MWh (straddle nameplates); **mass conservation exact: Δm_actual = ∫(prod−cons)dt = −8.646 kg, err 0.000 kg**. Tank movement is intentionally small (1000 kg tank, ~0.1–0.2 kg/s flows over 120 s) — direction + balance are validated here; the 8 h reliability sweep is Task 7.

---

### 3.13 `EMS` (Stateflow Layer-1, A6) — **the 12th and LAST module**

**Role:** Rule-based, 4-state **Stateflow** energy-management supervisor — the SLOW
layer (100 ms, §5.1). It decides *which mode* + *where energy is routed*; it does
NOT do fast f/V control (the GFM-VSC droop + A8 secondary already do, §3.5).
Powidian / Ipsakis hierarchical-supervisor logic (CLAUDE.md A6 L1, Rule 11).

| Port | Dir | Type | Unit | Source/Dest | Notes |
|---|---|---|---|---|---|
| **Inputs from globals (internal From blocks):** | | | | | |
| `f_meas`  | In (From) | Sim | Hz  | From `f_meas`  | df = f_meas − 50 |
| `SOC_ESS` | In (From) | Sim | 0–1 | From `SOC_ESS` | battery state |
| `SOC_H2`  | In (From) | Sim | 0–1 | From `SOC_H2`  | tank protection |
| `CB_PCC`  | In (From) | Sim | 0/1 | From `CB_PCC`  | island flag (1=grid, 0=island) |
| **Inputs from direct lines (subsystem Inports):** | | | | | |
| `In_P_pv_MW`   | In | Sim | MW | ← `PV/Out_P_pv_MW` | for P_net |
| `In_P_wind_MW` | In | Sim | MW | ← `Wind/Out_P_wind_MW` | for P_net |
| `In_P_load_MW` | In | Sim | MW | ← `MicrogridACBus/In_P_load_MW` (split) | for P_net |
| **Outputs (subsystem Outports):** | | | | | |
| `Out_Pref_ess_MW`  | Out | Sim | MW  | → `ESS/In_Pref_MW`          | [−80,+80], +=discharge |
| `Out_Pcmd_elec_MW` | Out | Sim | MW  | → `Electrolyzer/In_Pref_MW`| 0 or [5,40] |
| `Out_Pcmd_fc_MW`   | Out | Sim | MW  | → `PEMFC/In_Pref_MW`       | 0 or [2,30] |
| `Out_state`        | Out | Sim | 1..4| scope / debug              | 1=GRID 2=TRANS 3=STABLE 4=EMERG |

**AS-BUILT (build_ems.m v1, validated 2026-06-27) — 4-state Stateflow chart, Task 6.**
Thresholds (Ipsakis-style hysteresis) carried as Stateflow **Constant data** (visible
in the chart, M4). Chart `ActionLanguage=MATLAB`, `ChartUpdate=DISCRETE`,
`SampleTime=0.1` (M5). SIGNAL-LEVEL — no Simscape (M2), like H2Tank §3.12.
- **States & dispatch:**
  - **1 GRID_CONNECTED** (CB1=1): grid is buffer. P_net=P_pv+P_wind−P_load.
    Surplus → battery steered to its target band (charge while SOC<0.70), then
    *economic* electrolyzer at high SOC. Deficit → import from grid, **FC OFF**
    (preserve H2). EL on-hysteresis SOC≥0.80 / off SOC≤0.75 (+ surplus + tank<0.95).
  - **2 ISLAND_TRANSITION** (CB1 just 0): **FREEZE hydrogen** — `Pcmd_elec=0`,
    `Pref_ess=0` (VSC battery droop takes over), and **`Pcmd_fc` is left UNWRITTEN so
    the Stateflow output HOLDS its last value** (the idiomatic "hold FC"; τ_FC can't
    follow the transient). Exit: |Δf|<0.5 ∨ t_island>5 s → ISLAND_STABLE.
  - **3 ISLAND_STABLE** (CB1=0, settled): full coordination. **TIME-SCALE SPLIT** —
    `pnet_lpf` is a 1st-order LPF of P_net (τ=20 s); the SLOW part is served by
    hydrogen (EL/FC), the FAST residual by the battery (`Pref_ess = −P_net + Pelec −
    Pfc`, clamped ±80). EL hysteresis [0.80/0.75], FC hysteresis [0.25/0.30] (on at
    low SOC + deficit), tank guards [full 0.95 / empty 0.05].
  - **4 EMERGENCY** (|Δf|≥1.5 from any state): all resources to the rail —
    **sign-aware** (M6): under-frequency (df<0) → ESS +80, FC 30, **EL 0**;
    over-frequency → ESS −80, EL 40, FC 0. Exit |Δf|≤0.8 → GRID (if CB1=1) else STABLE.
- **Transition table** (ExecutionOrder: EMERGENCY entry highest, then CB, then settle):
  `GRID→EMERG[|Δf|≥1.5]/→TRANS[CB1<0.5]`; `TRANS→EMERG/→GRID[CB1≥0.5]/→STABLE[|Δf|<0.5∨t>5]`;
  `STABLE→EMERG/→GRID[CB1≥0.5]`; `EMERG→GRID[|Δf|≤0.8∧CB1≥0.5]/→STABLE[|Δf|≤0.8∧CB1<0.5]`.
- **Hysteresis (M3)** = chart Local latches `el_on`/`fc_on` with DIFFERENT on/off
  thresholds (anti-chatter; Red-Flag "state machine oscillation → use hysteresis").
- **Rule 4** single TOTAL clamp per device; **Rule 7** the SOC/tank guards + latch
  freezing. Dispatch outputs are SLOW setpoints; ESS does its own fast V_dc trim (§3.9).
- **Programmatic-Stateflow gotchas resolved (M1):** every action variable (incl.
  temporaries `Pnet`/`df`) must be declared as chart Data (no auto-create when built
  via the API); every **Local needs an explicit `InitialValue` (`'0'`)** or the strict
  "read-before-write" diagnostic errors on the persistent latches. See [[reference_stateflow_programmatic]].

**SCOPE NOTE (M8):** this module emits the **3 dispatch references + state id** (the
task's list). The VSC `f_ref`/`V_ref`/`mode` and the breaker `CB_PCC_cmd` stay at their
§5.3 constant defaults — they belong to the DEFERRED full-model wiring / island
session, not to building the EMS organ. EMS reads CB1 as an input (island flag);
the A8 secondary loop is always-on inside the VSC; islanding is an external scenario
event. So at master level the EMS Inports/Outports are built but **not yet wired to
the other modules** (consistent with the deferred DC-bus electrical wiring).

**Validated (build_ems_test.m, 2026-06-27; ISOLATED, signal-level, FixedStepDiscrete
@0.1 s; 4 globals supplied by test-side Goto blocks, P_pv/P_wind/P_load by Inports).
ALL 7 scenarios PASS:**
1. GRID + surplus(+100) + SOC 0.85 → **EL on (40 MW)**, FC 0, state 1.
2. GRID + deficit(−100) + SOC 0.50 → **FC off, EL 0, ESS 0** (grid imports), state 1.
3. Islanding CB 1→0 → **GRID@0 → TRANS@5.0 s → STABLE@8.1 s**; EL frozen=0 through
   TRANSITION; final STABLE Pess=+40 MW (covers the deficit).
4. ISLAND_STABLE + SOC 0.20 (low) + deficit → **FC on (30 MW)**, EL 0, Pess +70, state 3.
5. Hysteresis sweep SOC 0.72↔0.84 (surplus): SOC up-crossings of 0.80 = **4 = EL
   turn-ON events = 4** → one ON per up-crossing, **no chatter**.
6. |Δf|=1.6 Hz (under-freq) → **EMERGENCY: ESS +80, FC 30, EL 0**; recovers to GRID
   when f restored (GRID@0 → EMERG@2.0 → GRID@7.1 s).
7. Hysteresis 0.74↔0.76 (straddles 0.75 only, never reaches 0.80) → **EL never turns
   on (0 events)** — the deadband prevents chatter.

Master integrated (build_microgrid_eelib.m modules row 12): **12/12, smoke 10/10**.

---

## 4. CONNECTION MAP (visual)

```
                   ┌───────────────────────────────────────┐
                   │         TransmissionNetwork           │
                   │   Bus1(SM) ─ Bus4(slack) ─ Bus5(SM)   │
                   │            ╲   │   ╱                  │
                   │            Bus3(SM,load400 MW)        │
                   │                                       │
                   │       Elec_PCC_230kV ──┐              │
                   └────────────────────────┼──────────────┘
                                            │ 3-φ 230 kV
                                            ▼
                          ┌─────────────────────────────────┐
                          │            PCC                  │
                          │  HV ─[XFMR 33/230 Δ-Y]─ CB1 ─MV │
                          │                                 │
                          │  Goto: f_meas, V_pcc, CB_PCC    │
                          │  Out_P_pcc_MW ──► EMS           │
                          │              ▲                  │
                          │   In_CB_PCC_cmd ◄── EMS         │
                          └─────────────────────────────────┘
                                            │ 3-φ 33 kV
                                            ▼
                          ┌─────────────────────────────────┐
                          │       MicrogridACBus            │
                          │   Wye-Connected Variable Load   │
                          │   ◄─ P_load_MW, Q_load_MVAr     │
                          │                                 │
                          │   Elec_VSC_in ──┐               │
                          └─────────────────┼───────────────┘
                                            │ 3-φ 33 kV
                                            ▼
                          ┌─────────────────────────────────┐
                          │       GridFormingVSC            │
                          │  V-f droop (MATLAB Function)    │
                          │  ◄─ V_ref_pu, f_ref_Hz, mode    │
                          │  ─► P_vsc_MW, Q_vsc_MVAr        │
                          │                                 │
                          │   Elec_DC_1500V ──┐             │
                          └───────────────────┼─────────────┘
                                              │ DC 1500 V
                                              ▼
   ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌──────┐  ┌─────────┐
   │   PV    │  │  Wind   │  │  ESS    │  │ Electro │  │ FC   │  │ DCBus   │
   │ Cap 80  │  │ Cap 120 │  │ ±80 MW  │  │ 5-40MW  │  │2-30MW│  │ (node)  │
   │  MW     │  │  MW     │  │ 200 MWh │  │ τ=30s   │  │τ=5s  │  │ C_dc    │
   └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘  └──┬───┘  └────┬────┘
        └────────────┴────────────┴────────────┴──────────┴──────────┘
                                         DC bus 1500 V (all share node)
                                              │
                  mdot_H2 ◄────────────┐       │
                                       │       │
                                  ┌────┴───────┴────┐
                                  │     H2Tank      │
                                  │  1000 kg, 350bar│
                                  │  Goto: SOC_H2   │
                                  └─────────────────┘

   Globals (Goto/From):  f_meas, V_pcc, CB_PCC, SOC_ESS, SOC_H2
   Profile inputs (placeholder constants; From Workspace real_profiles later)
   Central controller:  EMS (4-state Stateflow, §3.13)
```

### 4.1 AS-BUILT FULL-MODEL WIRING (`wire_microgrid.m`, 2026-06-28)

The 12 modules are now connected into one running model by **`SIMULINK/wire_microgrid.m`**, called by the master after the modules + H2 chain. Groups (each guarded/idempotent):
- **G1 DC star** — PV/Wind/ESS/Electrolyzer/PEMFC/VSC DC ports → the 6 `DCBus` terminals (all one node → order irrelevant).
- **G2 AC chain** — `TransmissionNetwork`→`PCC`→`MicrogridACBus`→`GridFormingVSC` (230/33 kV electrical).
- **Solver Configuration** — the master's single `SolverConfig` is **branched onto a DC-bus node** (a conserving port accepts multiple lines; SolverConfig imposes nothing). The master created it but had deferred the wiring; once the modules form ONE physical network it needs exactly one Solver Configuration. (Do NOT branch the Electrical Reference onto a live rail — it would short it.)
- **G3 EMS commands** — `EMS` Out_Pref_ess/Pcmd_elec/Pcmd_fc → `ESS`/`Electrolyzer`/`PEMFC` In_Pref_MW.
- **G4 EMS inputs** — `PV`/`Wind` P → EMS; a single **`Src_Pload_MW` constant (300) SPLIT** to `MicrogridACBus/In_P_load_MW` and `EMS/In_P_load_MW` (§3.13).
- **G5 sources** — placeholder constants: PV irr/T, Wind v, bus Q; VSC `V_ref=1`/`f_ref=50`/`mode=1` (§5.3 defaults); `CB_PCC_cmd=1` (grid). All `Src_*` so the real_profiles swap is a one-liner.

**Algebraic-loop break (EMS, build_ems M9):** closing the loop creates a direct-feedthrough cycle EMS out → ESS/EL/FC ctrl → SOC_ESS/SOC_H2 Goto → EMS in (the ctrls update SOC from this step's command). Two **Memory** blocks on the EMS SOC_ESS/SOC_H2 inputs break it (rate-agnostic; EMS acts on the previous cycle's SOC — honest for a slow supervisor). f_meas/CB1 are not in the loop.

**Validated (build_fullmodel_gridtest.m, 2026-06-28; grid mode CB1=1, D_pu=60 test-only, 2 s, ode14x@50 µs, ~5 min wall): GRID-MODE INTEGRATION PASS.** f=50.00 Hz, V_pcc=0.994 pu, V_dc=1498 V (held), SOC flat, P_pv=21/P_wind=22 MW, Q_vsc=81 MVAr (VSC supplies load Q locally), P_pcc=298 MW (grid feeds the load), EMS in GRID (state 1) with dispatch reaching the modules, no NaN/Inf. The model **compiles** in the master smoke (update_diagram).

**⚠ FINDING — grid-mode DC export gap (R1/R4, NOT a wiring bug):** at PV 800 W/m² + Wind 9 m/s (≈127 MW) V_dc RAN AWAY to 82 kV: the grid-forming VSC (A2) exports ≈0 real power at nominal f in grid mode, so the DC renewable surplus exceeds ESS (±80) + electrolyzer (40) absorption → C_dc charges unbounded. The renewable placeholders are therefore sized to balance (PV 300 ≈20 MW + Wind 6 ≈22 MW < 80). Real fix (grid-mode renewable export / curtailment, or VSC V_dc-export mode) is a later session. **D_pu stays 1.5 in the saved model** (test overrides to 60 until the Task-5.2b damping fix).

---

## 5. CROSS-CUTTING CONVENTIONS

### 5.1 Sample times

| Layer | Sample time | Blocks |
|---|---|---|
| Electrical (Simscape) | 50 µs (continuous, then discretized by Solver Configuration) | All `~Elec~` |
| Control loops (droop, PI) | 1 ms | VSC vf_droop, ESS rate limiter |
| EMS | 100 ms | EMS chart |
| H2 dynamics | 100 ms | H2Tank integrator |
| Real profiles | 1 hour (resampled to 100 ms with hold) | From Workspace |

Set sample time explicitly on every Simulink block. Inherited (-1) is forbidden outside Simscape PS-Simulink converters.

### 5.2 Frequency measurement (`f_meas`)

Implementation choices, in increasing complexity:
1. **v1 (Task 5)**: In grid-forming mode, `f_meas = VSC f_cmd` (the VSC dictates frequency). In grid-following mode, `f_meas` from a simple PLL based on `atan2(V_q, V_d)` derivative.
2. **v2 (if HIL exposes issues)**: Three-Phase PLL block (build manually from Park transform + PI on V_q).

Document the choice inside the PCC subsystem with a sticky note.

### 5.3 Default values when EMS is in placeholder mode

When EMS is in its Task-5 stub state, the model must still simulate stably:
- `P_ess_ref_MW = 0` → battery idle
- `P_fc_ref_MW = 0` → FC off
- `P_elec_ref_MW = 5` → electrolyzer at minimum (avoids stack damage)
- `V_ref_pu = 1.00`, `f_ref_Hz = 50.0`, `VSC_mode = 1` (grid-forming)
- `CB_PCC_cmd = 1` (closed)

These defaults are constants inside the EMS subsystem until Task 6.

### 5.4 Naming hygiene

- Subsystem block names: `PascalCase` (e.g., `GridFormingVSC`)
- Port labels on subsystem boundary: as in tables above
- Internal block names: descriptive in English (`Vmeas3ph`, `RateLimiter_ESS`)
- Goto/From tag names: `snake_case` (`f_meas`, `SOC_ESS`)
- Workspace parameter names: as in `mg_params.mat` (`mg.branch.R_ohm`, etc.)

### 5.5 Solver configuration (single instance, top level)

```
nesl_utility/Solver Configuration   (one block, "Use local solver" OFF)
ee_lib/Connectors & References/Electrical Reference   (one per isolated network)
```
Model-level: `Solver=ode14x`, `SolverType=Fixed-step`, `FixedStep=50e-6`. (CLAUDE.md Rule 1, revised 2026-06-06: ode4 diverged on the stiff ideal-source network → implicit fixed-step ode14x.)

---

## 6. CHANGE LOG

| Date | Version | Change |
|---|---|---|
| 2026-06-28 | v1.15 | **FULL-MODEL WIRING — the 12 modules connected into one running model & grid-mode validated.** New `SIMULINK/wire_microgrid.m` (called by the master step [6c]): G1 DC star, G2 AC chain, Solver-Config branch onto a DC node, G3 EMS→ESS/EL/FC, G4 PV/Wind/P_load→EMS (single P_load constant split), G5 source constants (VSC refs, CB cmd, profile placeholders). New §4.1 documents it. **build_ems M9**: two Memory blocks on the EMS SOC inputs break the full-model algebraic loop (EMS↔ESS/EL/FC/Tank). Master gains a `update_diagram` compile check in the smoke + `arrangeSystem` layout pass. New `SIMULINK/build_fullmodel_gridtest.m` (grid mode, D_pu=60 test-only): **PASS** — f=50.00, V_pcc=0.994, V_dc=1498 V held, EMS GRID with dispatch reaching modules. ⚠ **Finding (R1/R4):** at PV 800/Wind 9 (127 MW) V_dc runs away (GFM-VSC exports ≈0 real power at nominal f in grid mode → DC surplus > ESS+EL absorption); renewable placeholders sized to balance (PV 300/Wind 6 ≈42 MW) until a grid-mode export/curtailment fix. Backup `microgrid_case5_backup.slx` kept. |
| 2026-06-27 | v1.14 | **`EMS` (build_ems.m v1) built & validated — the 12th and LAST module, Task 6** (A6 Layer-1). §3.13 rewritten "as built": 4-state **Stateflow** chart (GRID_CONNECTED / ISLAND_TRANSITION / ISLAND_STABLE / EMERGENCY), MATLAB action language, DISCRETE @100 ms, SIGNAL-LEVEL (no Simscape). Reads 4 globals via From (f_meas, SOC_ESS, SOC_H2, CB_PCC — EMS owns NO tag), P_pv/P_wind/P_load via Inports; outputs ESS/EL/FC dispatch refs + state id. Ipsakis hysteresis (EL on0.80/off0.75, FC on0.25/off0.30, tank full0.95/empty0.05) as chart **Constant data**; time-scale split (P_net LPF τ=20 s → H2 slow, battery fast); sign-aware EMERGENCY; TRANSITION freezes H2 (FC held via the unwritten-output idiom). New file `SIMULINK/build_ems_test.m` (7 scenarios, FixedStepDiscrete @0.1 s, test-side Goto for globals): **ALL 7 PASS** — EL-on, grid-import, islanding seq, FC-on, hysteresis no-chatter (4=4 up-crossings), EMERGENCY rail, deadband. Programmatic-Stateflow gotchas: declare ALL action vars incl. temporaries; every Local needs explicit `InitialValue='0'` or read-before-write errors on the latches. **Master 12/12, smoke 10/10.** SCOPE: VSC f_ref/V_ref/mode + CB_PCC_cmd stay at §5.3 defaults; EMS↔modules power wiring is the DEFERRED full-model step. |
| 2026-06-25 | v1.13 | **Hydrogen chain WIRED & validated** (integration step, no new module physics). EL/FC→H2Tank mdot signals connected at the master level (`build_microgrid_eelib.m` new step [6b/7], direct port-to-port — NOT Goto/From per §2): `Electrolyzer/Out_mdot_H2_kgs→In_mdot_prod_kgs`, `PEMFC/Out_mdot_H2_cons_kgs→In_mdot_cons_kgs`. §3.10/§3.11 Source/Dest notes updated "(WIRED)"; §3.12 validation block extended. New closed-loop test `SIMULINK/build_h2chain_test.m` (DCBus+EL+FC+H2Tank+TEST-ONLY 1500 V, no grid, 120 s, 3 phases): fills +2.17 / empties −5.04 / net −5.77 kg, SOC⊂[0.05,0.95], Faraday EL 22.0 / FC 50.7 kg/MWh, **mass conservation exact (err 0.000 kg)**. Master smoke 11/12, 10/10. Cleanups: `build_h2tank_test.m` misleading "V_tank unphysical/supersedes" fprintf corrected (param now 42.16 m³ = 23.7 kg/m³, NIST-consistent); `case5_to_simscape.m` `mg.h2.tank.SOC_init` 0.30→0.50 (consistency with hydrogen_system_params; not read by h2tank). |
| 2026-06-25 | v1.12 | **`H2Tank` (build_h2tank.m v1) built & validated** (Task 5.11 — the LAST organ module, 11/12); §3.12 rewritten "as built" — SIGNAL-LEVEL only (no electrical), forward-Euler mass-balance integrator (persistent `m`), SOC bounds [0.05,0.95]→m∈[50,950] kg with the clamp as anti-windup freeze (Rule 7/R1), **Z-corrected real-gas pressure** (Z=1.2 @350 bar) with **V_tank DERIVED ≈42.16 m³** so 1000 kg→350 bar (supersedes the unphysical `h2_params.tank.V_tank_m3=10`=100 kg/m³; derived = 23.7 kg/m³, NIST-consistent). Goto `SOC_H2` (H2Tank = §2 tag owner). Port names finalized: In_mdot_prod_kgs / In_mdot_cons_kgs (← EL `Out_mdot_H2_kgs` / FC `Out_mdot_H2_cons_kgs`), Out_m_H2_kg / Out_SOC_H2 / Out_P_tank_bar. New file `SIMULINK/build_h2tank_test.m` (fake-mdot, FixedStepDiscrete @0.1 s). Validation: fill→freeze@950, empty→freeze@50, balanced flat@500; P linear, 1000 kg=350.0 bar, ideal under-reads 17 %. ⚠ Flags: m0=500 vs param SOC_init=0.30; V_tank 10→42.2 to reconcile in hydrogen_system_params.m. EL/FC→tank wiring is a separate integration step. Master 11/12, smoke 10/10. |
| 2026-06-20 | v1.11 | **`PEMFC` (build_fc.m v1) built & validated** (Task 5.10); §3.11 rewritten "as built" — HYBRID PHYSICAL source-twin of the electrolyzer: Correa (2004) polarization (`fc_cell` local fn, V_cell DROPS with current), P→J bisection on the rising branch (P(J) non-monotone, peak ≈75 MW), Faraday H2 CONSUMPTION (current-based, η_util=0.98), τ=5 s BoP lag, SOURCE-direction CCS (I=+P/V_dc, boost). Shared scalars from `h2_params.fc` (Rule 8) + Correa coeffs/N_p=408 literals. Corrects the v1 consumption formula (`P·54.6/3600/η_FC` double-divided η; 54.6 already = 1000/(33.3·0.55)). New file `SIMULINK/build_fc_test.m`. Validation: τ≈5.6 s, V_stack 686→470→421 V (DROPS, opposite EL), consumption 50–56 kg/MWh (straddles 54.6), boost raises V_dc. Master 10/12, smoke 10/10. |
| 2026-06-20 | v1.10 | **`Electrolyzer` (build_electrolyzer.m v1) built & validated** (Task 5.9); §3.10 rewritten "as built" — HYBRID PHYSICAL: Ulleberg (2003) polarization (`electrolyzer_cell` local fn), P→I bisection inversion, Faraday H2 (current-based, η_F=0.99), τ=30 s BoP lag, LOAD-direction CCS (I=−P/V_dc). Shared scalars from `h2_params.elec` (Rule 8) + Ulleberg coeffs/A=15 m² literals. Corrects the v1 H2 formula (`P·21/3600·η_E` double-counted η; 21 kg/MWh already = η·1000/33.3). New file `SIMULINK/build_electrolyzer_test.m`. Validation: τ≈30.6 s, U_stack 492→664→729 V (polarization), H2 21.74 kg/MWh @ part-load (straddles h2_params 21.02). Master 9/12, smoke 10/10. |
| 2026-06-20 | v1.9 | **`Wind` (build_wind.m v1) built & validated** (Task 5.8); §3.8 rewritten "as built" — HYBRID PHYSICAL (Cp(λ,β) Heier `cp_lambda.m` → Cp_max; four-region power curve; averaged gen/rectifier), supersedes the "mirror of PV controllable source" plan. Same PV pattern: signal-level physics → ONE CCS, Inport In_v_wind_ms, offline-vs-ninja validation. New files: `PARAMS/build_wind_params.m`, `PARAMS/cp_lambda.m`, `SIMULINK/build_wind_test.m`. Validation: 4 regions correct (cut-in/cube/rated-plateau/cut-out); offline annual 320.1 vs ninja 333.6 GWh (0.959), shape corr 0.9913. Master 8/12, smoke 10/10. v_rated=10 m/s, η_wind=0.90 tuned to ninja profile. |
| 2026-06-20 | v1.8 | **`PV` (build_pv.m v1) built & validated** (Task 5.7); §3.7 rewritten "as built" — HYBRID PHYSICAL model (single-diode `pv_cell` + P&O `mppt_po` + averaged boost), supersedes the "controllable DC source from real_profiles" plan. RAW irradiance/temperature drive the physics; ninja `electricity` is validation-only. New files: `PARAMS/build_pv_params.m` (CSV→pv_data.mat + diode params→pv_params.mat, STC-scaled to 80 MW), `PARAMS/pv_iv.m` (diode, shared offline twin), `PARAMS/pv_mpp.m` (offline MPP sweep), `SIMULINK/build_pv_test.m`. Validation: MPPT lands on swept MPP at 4 conditions; offline annual 131.8 vs ninja 124.8 GWh (1.056), shape corr 0.9999, temp derating −15.6 %/30 °C. Master 7/12, smoke 10/10. Inports In_irr_Wm2/In_T_amb_C (P2). |
| 2026-06-20 | v1.7 | **`ESS` (build_ess.m v1) built & validated** (Task 5.6); §3.9 rewritten "as built" (H1–H7: signal-level Thevenin = one Controlled Current Source w/ verified port map, self-measured V_dc, Kp=0.2 not 0.5, slew on EMS FF only, Rule-4/7). Grid-connected test passes: V_dc held within ±5 % through a 300→320 MW load step, balance closes. **§3.5 note added — G8 VSC pre-sync sync breaker** `CB_sync` (the soft-start was a near-short on the energized bus; Task 5.5 missed it because the ideal DC source masked it). Two test-harness prerequisites recorded: transmission lightly-damped ~2–3 Hz mode → D_pu 30→60 (Task 5.2b); load step rate-limited ~50 ms. Also `build_transmission.m`: R_parasitic=1 Ω on the shunt PI caps (breaks an IC-solve singularity exposed once the ESS added states). |
| 2026-05-27 | v1   | Initial contract; covers Task 5 scope (build_microgrid_eelib.m + modules). |
| 2026-05-27 | v1.1 | C_farad verified in nF range (40–190 nF), PI-section C/2-each-end split spelled out (§3.1). DC-link voltage regulation assigned to **ESS** (§3.9); VSC has only a saturation backstop (§3.5). DCBus capacitor sizing rule added (§3.6, C_dc ≈ 30 mF). Breaker snubber/RC damper guidance added (§3.3). |
| 2026-05-28 | v1.2 | `TransmissionNetwork` (build_transmission.m v1) built and verified vs `runpf(case5)`: slack 1.0 pu exact, angles ±0.41°, magnitudes +0.6–2.7%. Added §3.1 critical note: slack `impedance_option='0'` mandatory (default '1' collapses the bus); verified ee_lib 3-phase port maps; PS-converter input filtering needed for explicit solvers. Logged the ode4-divergence / implicit-solver finding (Rule 1 open item). |
| 2026-06-06 | v1.3 | Rule 1 solver ode4→**ode14x** (§5.5); TransmissionNetwork magnitudes re-validated, gen-bus angles ~8° off (Task 5.2b). **`PCC` (build_pcc.m v1) built & validated** (§3.3 rewritten "as built"): native `Circuit Breaker (Three-Phase)` with inverted command, Δ(HV)/Yg(MV) transformer (vector-group decision D2 flagged), SRF-PLL on discrete 50 µs, Clarke V_to_pu / PQ_calc. Single Solver Configuration covers HV+MV (one magnetically-coupled network). |
| 2026-06-10 | v1.6 | **`GridFormingVSC` + `DCBus` (build_vsc.m / build_dcbus.m v1) built & validated** (Task 5.5). §3.5 rewritten "as built": AV-VSC verified port map (`LConn1=mod-wave PS, LConn2=AC 3φ, RConn1=DC+, RConn2=DC−`) and gain law **v=m·V_dc/2** → internal 0.69/33 kV Yg/Δ11 350 MVA step-up REQUIRED (leakage = coupling Z; Δ MV keeps A9 zig-zag the sole island ground); single 50 µs discrete `vf_droop` (Clarke PQ + LPF τ=10 ms, droop SLOPE form — the v1.1 "(1/R)" notation was unphysical, flagged G4 —, A8 secondary PIs w/ Rule-7 anti-windup, 200 ms soft-start, V_dc-adaptive modulation); **Unit Delay (1 sample) breaks the sensors→droop→m→network algebraic loop** (1st-order S-PS filter does not). §3.6 rewritten "as built": grounded-negative-rail convention, C_dc=30 mF IC 1500 V, verified Capacitor/Voltage-Sensor maps. Test (CB1 CLOSED, 300 MW, TEST-ONLY DC source): **V_pcc 0.945→1.0003 pu**, f=50.018 Hz, V_dc=1499.5 V, Q_vsc=+98 MVAr, P balance closes. Island test deferred. |
| 2026-06-08 | v1.5 | **`MicrogridACBus` + `ZigZagGrounding` (build_mg_acbus.m v1) built & validated** (Task 5.4). §3.4 rewritten "as built": load is `Wye-Connected Variable Load (lagging)` (`wye_series_RL`, P/Q PS inputs — the plain variant is resistive-only) with verified port map `LConn1=P, LConn2=Q, LConn3=3φ, RConn1=neutral`; constant-impedance behaviour noted; bus current sensor in series on the PCC feed. §3.4.1 rewritten "as built": **zig-zag = native `ee_lib/Passive/Transformers/Earthing Transformer`** (no shunt equivalent needed), port map `LConn1=3φ, RConn1=neutral`; 1-φ NGR resistor + 1-φ neutral current sensor (`LConn1=in, RConn2=out, RConn1=PS`); built as a NESTED subsystem tapped in parallel on the bus. Test (CB1 CLOSED, 300 MW @pf0.95): V_pcc=0.945 pu, P_pcc=+267.8 MW, f=49.97 Hz, I_neutral≈0 A. |
| 2026-06-08 | v1.4 | Documentation only (no model change). **A8 secondary control** note added to §3.5 (restore f→50 Hz / V→nominal on the battery-backed GFM VSC; two slow PI integrators, EMS-enabled; Rocabert 2012 / Guerrero). **A9 zig-zag grounding**: new subsystem **§3.4.1 `ZigZagGrounding`** (permanent 33 kV ground reference surviving CB1 open; resolves the D2 flag; optional NGR) + grounding note in §3.4. Both to be built in Task 5.4 (grounding) / Task 5.5 (secondary loop). Open risks **R1–R5** recorded in CLAUDE.md (DC-bus handover, CPL instability, low inverter fault current, scenario sizing < 80 MW deficit, averaged-model scope). Header bumped v1.2→v1.4 (v1.3 row pre-existed; header was stale). |
