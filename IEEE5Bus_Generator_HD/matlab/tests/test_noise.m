function [pass, msg] = test_noise()
%Olcum gurultusune dayaniklilik (LS). Dusuk iki seviye gecmeli, yuksek seviye raporlanir.
lv = [0 0; 1e-6 1e-4; 1e-5 1e-3; 1e-4 1e-2];
parts = ''; ok = true;
for i = 1:size(lv,1)
    o = case5SwingSim(struct('Tend', 10, 'pmFun', @(t) [1, 1 + 0.2*(t >= 1), 1], ...
        'noiseW', lv(i,1), 'noiseP', lv(i,2), 'seed', 7));
    k = 2; r = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 1);
    eH = abs(r.H-5.5)/5.5; eD = abs(r.D-1.5)/1.5;
    parts = [parts sprintf('[nw=%.0e,nP=%.0e: H=%.2f D=%.2f] ', lv(i,1), lv(i,2), r.H, r.D)]; %#ok<AGROW>
    if i <= 3 && (eH > 0.10 || eD > 0.40), ok = false; end
end
pass = ok; msg = parts;
end
