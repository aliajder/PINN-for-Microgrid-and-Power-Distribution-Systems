function [pass, msg] = test14_equilibrium()
%Bozucu yokken tum makineler w=1'de kalir ve Te=Tm.
o = case14SwingSim(struct('Tend', 5));
e1 = max(abs(o.w(:) - 1)); e2 = max(max(abs(o.Te - o.Tm)));
pass = e1 < 1e-8 && e2 < 1e-8;
msg = sprintf('max|w-1|=%.1e, max|Te-Tm|=%.1e', e1, e2);
end
