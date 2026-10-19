function [pass, msg] = test14_damping_form()
%Sabit terimli regresyon D*(w-1) formunu ayirt eder (D=2 olan makine 16).
S = load(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'mg14_params.mat')); mg = S.mg14;
base = mg.bus.Pd_MW(1:14) + 1i*mg.bus.Qd_MVAr(1:14);
ex = zeros(14,1); ex([3 4 9]) = 0.2*base([3 4 9]);
o = case14SwingSim(struct('Tend', 10, 'loadFun', @(t) ex*(t >= 1)));
k = 2; f = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), NaN);
bad = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 0);
pass = abs(f.c) < 0.1 * o.D(k) && bad.D < 0.3 * o.D(k);
msg = sprintf('c=%.3g (D=%g); yanlis D*w formu D=%.3f', f.c, o.D(k), bad.D);
end
