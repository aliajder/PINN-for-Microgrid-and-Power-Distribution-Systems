function [pass, msg] = test_equilibrium()
%Bozucu yokken sistem denge noktasinda kalmali (w=1, Te=Tm).
o = case5SwingSim(struct('Tend', 5));
e1 = max(abs(o.w(:) - 1)); e2 = max(max(abs(o.Te - o.Tm)));
pass = e1 < 1e-8 && e2 < 1e-8;
msg = sprintf('max|w-1|=%.2e, max|Te-Tm|=%.2e', e1, e2);
end
