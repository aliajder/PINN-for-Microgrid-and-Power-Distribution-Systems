function [pass, msg] = test_disturbance_types()
%Farkli bozucu tipleri (Pm basamak/rampa, yuk basamagi/darbesi) ayni H,D'yi vermeli.
base = @(t) [0 0 0];
SL = @(t, extra) complex([0;0;300;400;0] + extra, [0;0;98.61;131.47;0] + 0.329*extra);
cases = {
  'pm_step',   struct('pmFun', @(t) [1, 1 + 0.2*(t >= 1), 1])
  'pm_ramp',   struct('pmFun', @(t) [1, 1 + 0.2*min(max(t-1,0)/2,1), 1])
  'load3_step',struct('loadFun', @(t) SL(t, [0;0;100*(t>=1);0;0]))
  'load5_pulse',struct('loadFun', @(t) SL(t, [0;0;0;0;150*(t>=1 && t<3)]))
  'pcc_load2', struct('loadFun', @(t) SL(t, [0;120*(t>=1);0;0;0]))};
Hs = zeros(size(cases,1),1); Ds = Hs; maxErrH = 0; maxErrD = 0; parts = '';
for i = 1:size(cases,1)
    o = case5SwingSim(setfield(cases{i,2}, 'Tend', 10)); k = 2;
    r = estimateHD_LS(o.t, o.Tm(:,k), o.Te(:,k), o.w(:,k), 1);
    maxErrH = max(maxErrH, abs(r.H-5.5)/5.5); maxErrD = max(maxErrD, abs(r.D-1.5)/1.5);
    parts = [parts sprintf('%s:H=%.3f,D=%.3f ', cases{i,1}, r.H, r.D)]; %#ok<AGROW>
end
pass = maxErrH < 0.05 && maxErrD < 0.10;
msg = [parts sprintf('| max hata H=%.1f%% D=%.1f%%', 100*maxErrH, 100*maxErrD)];
end
