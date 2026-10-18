function [pass, msg] = test_inputs_consistency()
%Girdi dosyalari (mg_params.mat) ve simulatorun kendi icinde tutarliligi.
S = load(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'inputs', 'mg_params.mat')); mg = S.mg;
ok = isfield(mg, 'gen') && isfield(mg, 'branch') && isfield(mg, 'bus') && mg.slack.bus == 4;
ok = ok && isequal(mg.gen.bus(:)', [1 1 3 4 5]) && numel(mg.branch.from) == 6;
o = case5SwingSim(struct('Tend', 0.1));
rel = abs(o.Pe0 - o.Pg_pu) ./ o.Pg_pu;
pass = ok && all(rel < 0.30);
msg = sprintf('Pe0/S=[%s], Pg/S=[%s], max bagil fark %.1f%% (Bus2 MG yuku yok, sabit-Z yuk)', ...
    num2str(o.Pe0', '%.3f '), num2str(o.Pg_pu', '%.3f '), 100*max(rel));
end
