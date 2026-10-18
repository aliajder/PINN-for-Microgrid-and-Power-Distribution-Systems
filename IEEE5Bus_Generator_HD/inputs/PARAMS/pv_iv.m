function I = pv_iv(V, G, T_amb, p)
%% pv_iv.m — single-diode PV I–V curve (SHARED: Simulink pv_cell + offline)
%#codegen
% One equivalent PV array (NOT cell-by-cell), lumped and scaled to p.P_rated_W.
%
%   I = I_ph − I_0·(exp(V/a) − 1)
%   I_ph   = (G/1000)·I_sc_stc·(1 + alpha_Isc·(T_cell−25))     photocurrent ∝ irradiance
%   a      = N_s·n·k·T_K/q                                     modified thermal voltage
%   V_oc(T)= V_oc_stc·(1 + beta_Voc·(T_cell−25))               open-circuit V drops with heat
%   I_0    = I_ph/(exp(V_oc(T)/a) − 1)                         back-out so curve hits (V_oc,0)
%   T_cell = T_amb + (NOCT−20)/800·G                           NOCT cell-temperature rise
%
% Inputs : V [V] operating voltage (scalar in Simulink, vector for an offline
%          sweep), G [W/m^2] plane-of-array irradiance, T_amb [°C] ambient,
%          p = parameter struct from build_pv_params.m.
% Output : array current I [A] (same size as V).
%
% Property used for STC scaling (build_pv_params.m): scaling I_sc_stc by s
% scales I_ph and I_0 by s, so I(V) and P(V)=V·I scale EXACTLY by s with V_mpp
% unchanged → P_mpp ∝ I_sc_stc (one-shot nameplate scaling).
%
% SINGLE SOURCE OF TRUTH for the diode: the Simulink 'pv_cell' block calls this
% same function, and the offline annual validation (build_pv_test.m) calls it —
% so the HIL model and the energy check cannot drift. (build_pv.m, Task 5.7.)

q = 1.602176634e-19;   % elementary charge [C]
k = 1.380649e-23;      % Boltzmann constant [J/K]

T_cell = T_amb + (p.NOCT - 20)/800 * G;        % [°C]
TK     = T_cell + 273.15;                      % [K]
a      = p.N_s * p.n * k * TK / q;             % [V] modified thermal voltage

I_ph   = (G/1000) * p.I_sc_stc * (1 + p.alpha_Isc*(T_cell - 25));

if I_ph <= 0                                   % dark / no irradiance
    I = zeros(size(V));
    return;
end

V_oc_T = p.V_oc_stc * (1 + p.beta_Voc*(T_cell - 25));
I_0    = I_ph / (exp(V_oc_T/a) - 1);           % reverse saturation (fitted to V_oc)
I      = I_ph - I_0 .* (exp(V./a) - 1);
end
