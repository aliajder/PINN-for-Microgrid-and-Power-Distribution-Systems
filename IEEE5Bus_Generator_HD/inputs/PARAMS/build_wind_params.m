%% build_wind_params.m  (Task 5.8 — wind turbine params + real data)
% Mirrors build_pv_params.m. Builds two DATA/ files:
%   wind_data.mat   — hourly wind speed v [m/s] timeseries (ninja RAW input).
%   wind_params.mat — turbine params (struct `wd`): Cp_max from cp_lambda(λ,0),
%                     rotor area A scaled so the cube region hits 120 MW at
%                     v_rated, four-region speeds, efficiencies.
%
% HYBRID (Task 5.8): RAW wind speed drives a physical Cp(λ,β)+power-curve
% engine. Renewables.ninja `electricity` (a Vestas V90 2000 curve) is the
% VALIDATION reference only — NOT a model input. Run after build_pv_params.m.

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
csv  = fullfile(root, 'DATA', 'ninja_wind_41.0286_28.8901_uncorrected.csv');
addpath(here);   % cp_lambda on path

%% ─── Read ninja CSV (3 '#'-comment lines, then header) ─────────────────────
opts = detectImportOptions(csv, 'CommentStyle', '#');
T    = readtable(csv, opts);
v_ms    = T.wind_speed;          % RAW physical input [m/s]
elec_pu = T.electricity;         % per-unit (capacity=1) — validation ref only
nH      = height(T);

%% ─── Hourly timeseries for From Workspace (ZOH) ────────────────────────────
t_s  = (0:nH-1).' * 3600;
v_ts = timeseries(v_ms, t_s, 'Name', 'v_wind_ms');
save(fullfile(root,'DATA','wind_data.mat'), 'v_ts', 'elec_pu');

%% ─── Turbine parameters (one equivalent aggregate turbine, % TUNED) ────────
wd = struct();
wd.rho       = 1.225;      % air density                [kg/m^3]
wd.cut_in    = 3.0;        % cut-in wind speed          [m/s]  % TUNED
wd.v_rated   = 10.0;       % rated wind speed           [m/s]  % TUNED (cube region → P_rated here;
                           %                                   10 m/s best matches ninja's profile/energy)
wd.cut_out   = 25.0;       % cut-out (safety stop)      [m/s]  % TUNED (data max 16.9 → never binds in 2025)
wd.P_rated_W = 120e6;      % nameplate                  [W]   (A4: wind 120 MW)
wd.eta_gen   = 0.95;       % generator + rectifier      [-]   (spec)
wd.eta_wind  = 0.90;       % wake + availability derate [-]   % TUNED (≈ PV eta_sys; calibrate to ninja annual)
wd.V_dc_nom  = 1500;       % bus the DC-DC feeds        [V]
wd.V_floor   = 750;        % min V_dc for I=P/V_dc      [V]

% Cp_max from the analytic Cp(λ,0) — the optimal-λ operating point.
lam = linspace(2, 15, 1000);
Cpv = cp_lambda(lam, 0);
[wd.Cp_max, im] = max(Cpv);
wd.lambda_opt   = lam(im);

% Rotor area so the cube region reaches P_rated exactly at v_rated:
%   P_rated = Cp_max · 0.5·ρ·A · v_rated^3
wd.A = wd.P_rated_W / (wd.Cp_max * 0.5 * wd.rho * wd.v_rated^3);
wd.R = sqrt(wd.A/pi);

save(fullfile(root,'DATA','wind_params.mat'), 'wd');

%% ─── Summary ──────────────────────────────────────────────────────────────
fprintf('\n=============== build_wind_params (Task 5.8) ===============\n');
fprintf(' CSV: %d hours | v[m/s] %.2f..%.2f (mean %.2f) | turbine ref: Vestas V90 2000\n', ...
    nH, min(v_ms), max(v_ms), mean(v_ms));
fprintf(' ninja electricity (per-unit ref): annual %.0f kWh/kW → %.1f GWh @120 MW (CF %.3f)\n', ...
    sum(elec_pu), sum(elec_pu)*120e3/1e6, mean(elec_pu));
fprintf(' Cp_max = %.4f at lambda_opt = %.2f (β=0)\n', wd.Cp_max, wd.lambda_opt);
fprintf(' four-region: cut_in=%.1f, v_rated=%.1f, cut_out=%.1f m/s | A=%.0f m^2 (R_eq=%.0f m)\n', ...
    wd.cut_in, wd.v_rated, wd.cut_out, wd.A, wd.R);
fprintf(' losses: eta_gen=%.2f · eta_wind=%.2f → delivered at rated ≈ %.1f MW\n', ...
    wd.eta_gen, wd.eta_wind, wd.P_rated_W*wd.eta_gen*wd.eta_wind/1e6);
fprintf(' saved DATA/wind_data.mat (v_ts,elec_pu) + DATA/wind_params.mat (wd)\n');
fprintf('============================================================\n');
