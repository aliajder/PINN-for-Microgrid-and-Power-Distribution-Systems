function [pass, msg] = test14_data()
%mg14_params.mat: makale (Tablo I) ve topoloji (19 bara) ile tutarlilik.
S = load(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'mg14_params.mat')); mg = S.mg14;
ok = numel(mg.bus.id) == 19 && numel(mg.branch.from) == 25 && numel(mg.gen.H) == 5;
ok = ok && isequal(mg.gen.unit_bus', [15 16 17 19 18]) && isequal(mg.gen.sys_bus', [1 2 3 6 8]);
ok = ok && max(abs(mg.gen.H' - [2.656 4.985 1.520 1.200 1.200])) < 1e-12;
ok = ok && isequal(mg.gen.D', [2 2 0 0 0]) && isequal(mg.gen.S_MVA', [448 100 40 25 25]);
ok = ok && isequal(mg.bus.Vbase_kV(15:19)', [22 13.8 13.8 13.8 13.8]);
ok = ok && all(mg.bus.Vbase_kV(1:5) == 220) && all(mg.bus.Vbase_kV(6:14) == 132);
loadMW = sum(mg.bus.Pload_only_MW(1:14)); loadMVAr = sum(mg.bus.Qd_MVAr(1:14) + 0) ;
ok = ok && abs(loadMW - 259) < 1e-9 && sum(mg.bus.Pload_only_MW > 0) == 11;
nTx = sum(mg.branch.isXfmr(1:20));
ok = ok && nTx == 3 && mg.microgrid.pcc_bus == 19 && mg.bus.Vbase_kV(19) == 13.8;
ok = ok && mg.pcc.V_hv_LL == 13.8e3 && mg.pcc.V_mv_LL == 33e3;
ok = ok && mg.check.maxPe_err_MW < 1e-3 && mg.check.maxV_err_pu < 1e-6;
pass = ok;
msg = sprintf('19 bara, 25 dal, yuk %.1f MW (11 yuk), 3 trafo, max|Pe0-Pg|=%.1e MW, max|dV|=%.1e pu', ...
    loadMW, mg.check.maxPe_err_MW, mg.check.maxV_err_pu);
end
