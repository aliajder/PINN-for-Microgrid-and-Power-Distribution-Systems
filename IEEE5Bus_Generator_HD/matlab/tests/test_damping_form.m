function [pass, msg] = test_damping_form()
%Sabit terimli regresyon sonucu D*(w-1) formunu ayirt etmeli (yanlis form D'yi kaybeder).
o = case5SwingSim(struct('Tend', 10, 'pmFun', @(t) [1, 1 + 0.2*(t >= 1), 1]));
k = 2; D = o.D(k);
f = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), NaN);
bad = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 0);
good = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 1);
pass = abs(f.c) < 0.1*D && abs(good.D - D)/D < 0.05 && bad.D < 0.3*D;
msg = sprintf('c=%.3g (D=%.2f), D*(w-1) formu D=%.3f, yanlis D*w formu D=%.3f', f.c, D, good.D, bad.D);
end
