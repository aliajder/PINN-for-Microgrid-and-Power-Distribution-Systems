function [pass, msg] = test14_paper_event()
%Makale olayi: t=1 s'de Bara 3, 4, 9 yukleri %20 artar (toplam 34.3 MW). Sistem kararli kalmali.
S = load(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'mg14_params.mat')); mg = S.mg14;
base = mg.bus.Pd_MW(1:14) + 1i*mg.bus.Qd_MVAr(1:14);
ex = zeros(14,1); ex([3 4 9]) = 0.2*base([3 4 9]);
o = case14SwingSim(struct('Tend', 20, 'loadFun', @(t) ex*(t >= 1)));
stepMW = sum(real(ex));
dev = o.w - mean(o.w, 2);
m1 = o.t >= 1 & o.t < 4; m2 = o.t >= 16;
spread1 = max(max(dev(m1,:))) - min(min(dev(m1,:)));
spread2 = max(max(dev(m2,:))) - min(min(dev(m2,:)));
mA = mean(mean(o.w(o.t >= 12 & o.t < 16, :))); mB = mean(mean(o.w(m2, :)));
sp = max(abs(o.w(:) - 1));
pass = abs(stepMW - 34.3) < 0.05 && ~any(isnan(o.w(:))) && sp < 0.06 ...
    && spread2 <= 1.2*spread1 && abs(mB - mA) < 1e-3;
msg = sprintf('adim %.2f MW, max|w-1|=%.4f, makineler arasi sapma %.1e -> %.1e (buyumuyor), ort. w 12-16 s: %.4f, 16-20 s: %.4f', ...
    stepMW, sp, spread1, spread2, mA - 1, mB - 1);
end
