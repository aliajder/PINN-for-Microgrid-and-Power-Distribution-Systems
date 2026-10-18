function [Pmpp, Vmpp, Impp] = pv_mpp(G, T_amb, p)
%% pv_mpp.m — offline MPP of the single-diode array via a voltage sweep
% Used by build_pv_params.m (STC nameplate scaling) and build_pv_test.m
% (annual / daily energy validation against Renewables.ninja). The Simulink
% model finds the SAME point with a Perturb & Observe MPPT; the test
% cross-checks that P&O lands on this swept optimum.
%
% Inputs : G [W/m^2], T_amb [°C], p = params. Output : DC MPP (PRE-converter).
%   Pmpp [W], Vmpp [V], Impp [A].

if G <= 1
    Pmpp = 0; Vmpp = 0; Impp = 0;
    return;
end

T_cell = T_amb + (p.NOCT - 20)/800 * G;
V_oc_T = p.V_oc_stc * (1 + p.beta_Voc*(T_cell - 25));

V = linspace(0, max(V_oc_T, 1), 600);
I = pv_iv(V, G, T_amb, p);
I = max(I, 0);
P = V .* I;
[Pmpp, idx] = max(P);
Vmpp = V(idx);
Impp = I(idx);
end
