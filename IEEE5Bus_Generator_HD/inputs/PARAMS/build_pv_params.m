%% build_pv_params.m  (Task 5.7 — PV physical-model parameters + real data)
% Builds two .mat files in DATA/:
%   pv_data.mat   — hourly irradiance G [W/m^2] and ambient T [°C] timeseries
%                   (Renewables.ninja RAW input) for From Workspace / ZOH.
%   pv_params.mat — single-diode array params (struct `pv`), nameplate-scaled
%                   so the STC (G=1000, T=25) DC MPP = 80 MW (A-architecture).
%
% Run order: after case5_to_simscape.m / hydrogen_system_params.m, before
% build_pv.m. Companion physics: pv_iv.m (diode), pv_mpp.m (offline MPP sweep).
%
% HYBRID approach (Task 5.7): the RAW irradiance + temperature drive a physical
% single-diode engine. Renewables.ninja's processed `electricity` column is NOT
% used as an input — it is only the VALIDATION reference (build_pv_test.m).

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
csv  = fullfile(root, 'DATA', 'ninja_pv_41.0286_28.8901_uncorrected.csv');
addpath(here);   % pv_iv / pv_mpp on path

%% ─── Read Renewables.ninja CSV (3 '#'-comment lines, then header) ──────────
opts = detectImportOptions(csv, 'CommentStyle', '#');
T    = readtable(csv, opts);
% RAW physical inputs. irradiance columns are normalized (kW/m^2) → ×1000 = W/m^2.
G_Wm2  = (T.irradiance_direct + T.irradiance_diffuse) * 1000;   % plane-of-array [W/m^2]
T_amb  = T.temperature;                                         % ambient [°C]
elec_pu = T.electricity;                                        % per-unit (capacity=1) — validation ref only
nH      = height(T);                                            % 8760

%% ─── Hourly timeseries for From Workspace (ZOH in the model) ───────────────
t_s  = (0:nH-1).' * 3600;                  % hourly grid in seconds
G_ts = timeseries(G_Wm2, t_s, 'Name', 'irr_Wm2');
T_ts = timeseries(T_amb, t_s, 'Name', 'T_amb_C');
save(fullfile(root,'DATA','pv_data.mat'), 'G_ts', 'T_ts', 'elec_pu');

%% ─── Single-diode array parameters (crystalline-Si, % TUNED) ───────────────
pv = struct();
pv.V_oc_stc  = 900;        % array open-circuit voltage at STC   [V]  (utility string, < 1500 V bus)
pv.N_s       = 1500;       % cells in series (≈0.6 V/cell)        [-]
pv.n         = 1.3;        % diode ideality factor               [-]  % TUNED
pv.alpha_Isc = +0.0005;    % I_sc temp. coeff.  (+0.05 %/°C)     [1/°C] % TUNED c-Si
pv.beta_Voc  = -0.0040;    % V_oc temp. coeff.  (−0.40 %/°C)     [1/°C] % TUNED c-Si
pv.NOCT      = 45;         % nominal op. cell temp.              [°C]
pv.eta_dcdc  = 0.98;       % averaged DC-DC boost efficiency     [-]
pv.eta_sys   = 0.80;       % BoS/soiling/wiring/mismatch/avail.  [-]  performance-ratio derate (temperature is
                           %                                         modelled separately in the diode); CALIBRATED:
                           %                                         lands annual energy within ~6% of Renewables.ninja
pv.P_rated_W = 80e6;       % DC nameplate = STC MPP target       [W]  (A-architecture, R4)
pv.V_dc_nom  = 1500;       % bus the DC-DC boosts to             [V]  (A1)
pv.V_floor   = 750;        % min V_dc for the I=P/V_dc division  [V]

% Nameplate scaling: P_mpp ∝ I_sc_stc exactly (see pv_iv.m), so one shot.
pv.I_sc_stc = 1;
Pu = pv_mpp(1000, 25, pv);                 % unit-I_sc MPP at STC
pv.I_sc_stc = pv.P_rated_W / Pu;           % scale so STC MPP = 80 MW
[Pmpp0, Vmpp0, Impp0] = pv_mpp(1000, 25, pv);

save(fullfile(root,'DATA','pv_params.mat'), 'pv');

%% ─── Summary + STC sanity ─────────────────────────────────────────────────
fprintf('\n=============== build_pv_params (Task 5.7) ===============\n');
fprintf(' CSV: %d hours | G[W/m2] 0..%.0f | T[°C] %.1f..%.1f (mean %.1f)\n', ...
    nH, max(G_Wm2), min(T_amb), max(T_amb), mean(T_amb));
fprintf(' ninja electricity (per-unit, validation ref): annual %.1f kWh/kW → %.1f GWh @80 MW\n', ...
    sum(elec_pu), sum(elec_pu)*80e3/1e6);
fprintf(' diode array: V_oc_stc=%g V, N_s=%g, n=%.2f, I_sc_stc=%.0f A (scaled)\n', ...
    pv.V_oc_stc, pv.N_s, pv.n, pv.I_sc_stc);
fprintf(' STC MPP check (G=1000,T=25): P=%.2f MW, V_mpp=%.1f V, I_mpp=%.0f A  (target 80 MW)\n', ...
    Pmpp0/1e6, Vmpp0, Impp0);
fprintf(' losses: eta_dcdc=%.2f · eta_sys=%.2f → delivered STC ≈ %.1f MW\n', ...
    pv.eta_dcdc, pv.eta_sys, Pmpp0*pv.eta_dcdc*pv.eta_sys/1e6);
fprintf(' saved DATA/pv_data.mat (G_ts,T_ts,elec_pu) + DATA/pv_params.mat (pv)\n');
fprintf('==========================================================\n');
