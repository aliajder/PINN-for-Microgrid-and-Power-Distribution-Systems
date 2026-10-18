function [pass, msg] = test_csv_roundtrip()
%CSV yazma/okuma ve Simulink log donusumu.
o = case5SwingSim(struct('Tend', 5, 'pmFun', @(t) [1, 1 + 0.2*(t >= 1), 1]));
f = [tempname() '.csv']; exportGeneratorCsv(o, 2, f);
[t, Tm, Te, w] = loadGeneratorCsv(f);
e1 = max(abs(w - o.w(:,2))) + max(abs(Te - o.Te(:,2)));
f2 = [tempname() '.csv'];
S = o.S_MW(2);
simulinkLogToCsv(o.t, o.Tm(:,2)*S, o.Te(:,2)*S, o.w(:,2)*50, S, 50, f2);
[~, Tm2, Te2, w2] = loadGeneratorCsv(f2);
e2 = max(abs(Te2 - o.Te(:,2))) + max(abs(w2 - o.w(:,2)));
delete(f); delete(f2);
pass = numel(t) == numel(o.t) && e1 < 1e-6 && e2 < 1e-6;
msg = sprintf('csv hata=%.1e, simulink-donusum hata=%.1e', e1, e2);
end
