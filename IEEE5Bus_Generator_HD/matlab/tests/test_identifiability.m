function [pass, msg] = test_identifiability()
%Hiz neredeyse degismiyorsa (cok kucuk bozucu) tahmin 'tanimlanamaz' diye isaretlenmeli.
o1 = case5SwingSim(struct('Tend', 10, 'pmFun', @(t) [1, 1 + 1e-6*(t >= 1), 1]));
o2 = case5SwingSim(struct('Tend', 10, 'pmFun', @(t) [1, 1 + 0.2*(t >= 1), 1]));
r1 = estimateHD_LS(o1.t, o1.Tm(:,2), o1.Te(:,2), o1.w(:,2), 1);
r2 = estimateHD_LS(o2.t, o2.Tm(:,2), o2.Te(:,2), o2.w(:,2), 1);
pass = ~r1.identifiable && r2.identifiable;
msg = sprintf('kucuk bozucu: wSpan=%.1e identifiable=%d; normal: wSpan=%.1e identifiable=%d', ...
    r1.wSpan, r1.identifiable, r2.wSpan, r2.identifiable);
end
