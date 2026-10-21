function [pass, msg] = test_pinn()
%PINN tahmini dogru bolgeye gelmeli (H ~%10, D ~%60). LS kadar hassas degildir; D zayif tanimlidir.
setupGeneratorPaths();
o = case5SwingSim(struct('Tend', 10, 'pmFun', @(t) [1, 1 + 0.2*(t >= 1), 1]));
rng(3); k = 2;
r = estimateHD_PINN(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k));
eH = abs(r.H-5.5)/5.5; eD = abs(r.D-1.5)/1.5;
pass = eH < 0.10 && eD < 0.60;
msg = sprintf('PINN H=%.3f (hata %.1f%%), D=%.3f (hata %.1f%%), nFourier=%d', r.H, 100*eH, r.D, 100*eD, r.model.ctx.sizes(1)/2 - 0.5);
end
