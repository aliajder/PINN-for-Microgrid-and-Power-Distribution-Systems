function [pass, msg] = test14_noise()
%Olcum gurultusu (unite 16, D=2): dusuk iki seviye gecmeli, yuksek seviye raporlanir.
S = load(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'mg14_params.mat')); mg = S.mg14;
base = mg.bus.Pd_MW(1:14) + 1i*mg.bus.Qd_MVAr(1:14);
ex = zeros(14,1); ex([3 4 9]) = 0.2*base([3 4 9]);
lv = [0 0; 1e-6 1e-4; 1e-5 1e-3; 1e-4 1e-2];
parts = ''; ok = true;
for i = 1:size(lv,1)
    o = case14SwingSim(struct('Tend', 10, 'loadFun', @(t) ex*(t >= 1), 'noiseW', lv(i,1), 'noiseP', lv(i,2), 'seed', 7));
    k = 2; r = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 1);
    parts = [parts sprintf('[nw=%.0e nP=%.0e: H=%.2f D=%.2f] ', lv(i,1), lv(i,2), r.H, r.D)]; %#ok<AGROW>
    if i <= 3 && (abs(r.H-o.H(k))/o.H(k) > 0.10 || abs(r.D-o.D(k))/o.D(k) > 0.40), ok = false; end
end
pass = ok; msg = parts;
end
