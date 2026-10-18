function [pass, msg] = test_der_independence()
%Bus 2'deki (mikro sebeke/DER) guc dalgalanmasi jeneratorun kendi H,D'sini degistirmemeli.
SL = @(t, p2) complex([0; p2; 300; 400; 0], [0; 0.329*p2; 98.61; 131.47; 0]);
cases = {
  'pcc_step_120MW',  @(t) SL(t, 120*(t >= 1))
  'pcc_sin_60MW_0.3Hz', @(t) SL(t, 60*sin(2*pi*0.3*max(t-1,0))*(t >= 1))
  'pcc_pulse_-80MW', @(t) SL(t, -80*(t >= 1 && t < 4))};
maxH = 0; maxD = 0; parts = '';
for i = 1:size(cases,1)
    o = case5SwingSim(struct('Tend', 12, 'loadFun', cases{i,2})); k = 2;
    r = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 1);
    maxH = max(maxH, abs(r.H-5.5)/5.5); maxD = max(maxD, abs(r.D-1.5)/1.5);
    parts = [parts sprintf('%s:H=%.3f,D=%.3f ', cases{i,1}, r.H, r.D)]; %#ok<AGROW>
end
pass = maxH < 0.05 && maxD < 0.10;
msg = [parts sprintf('| max hata H=%.1f%% D=%.1f%%', 100*maxH, 100*maxD)];
end
