function [pass, msg] = test14_ls_all_machines()
%Uc bozucu (makale yuk olayi, Pm basamagi, MG yuku) x 5 makine: LS ile H ve D geri bulunur.
S = load(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'mg14_params.mat')); mg = S.mg14;
base = mg.bus.Pd_MW(1:14) + 1i*mg.bus.Qd_MVAr(1:14);
ex = zeros(14,1); ex([3 4 9]) = 0.2*base([3 4 9]);
sc = {'paper', struct('Tend', 10, 'loadFun', @(t) ex*(t >= 1));
      'pm_step', struct('Tend', 10, 'pmFun', @(t) [1, 1 + 0.3*(t >= 1), 1, 1, 1]);
      'mg_pcc_30MW', struct('Tend', 10, 'loadFun', @(t) [zeros(5,1); 30*(t >= 1); zeros(8,1)])};
maxH = 0; maxD = 0; ok = true;
for i = 1:size(sc,1)
    o = case14SwingSim(sc{i,2});
    for k = 1:5
        r = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 1);
        eH = abs(r.H - o.H(k))/o.H(k);
        if o.D(k) > 0, eD = abs(r.D - o.D(k))/o.D(k); else, eD = abs(r.D)/2; end
        maxH = max(maxH, eH); maxD = max(maxD, eD);
        ok = ok && r.identifiable;
    end
end
pass = ok && maxH < 0.05 && maxD < 0.05;
msg = sprintf('3 senaryo x 5 makine: en kotu H hatasi %.2f%%, D hatasi %.2f%% (D=0 icin |D|/2)', 100*maxH, 100*maxD);
end
