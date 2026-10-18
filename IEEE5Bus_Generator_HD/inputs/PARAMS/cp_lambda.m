function Cp = cp_lambda(lambda, beta)
%% cp_lambda.m — analytic power coefficient Cp(λ,β)  (Heier model)
%#codegen
% Standard turbine Cp curve (Heier / Slootweg). λ = tip-speed ratio,
% β = pitch angle [deg]. Used by build_wind_params.m to find Cp_max at the
% optimal λ (β=0); the averaged turbine then operates AT that optimum in the
% cube region (the wind analogue of MPPT) and pitches (β>0 → Cp<Cp_max) to
% hold P_rated above rated wind. Shared offline; the Simulink wind_turbine
% uses the resulting scalar Cp_max. (Task 5.8.)

c1 = 0.5176; c2 = 116; c3 = 0.4; c4 = 5; c5 = 21; c6 = 0.0068;   % % TUNED (generic)
inv_li = 1./(lambda + 0.08*beta) - 0.035./(beta.^3 + 1);
Cp = c1*(c2*inv_li - c3*beta - c4).*exp(-c5*inv_li) + c6*lambda;
Cp = max(Cp, 0);
end
