function res = estimateGeneratorFromCsv(csvPath, opts)
%ESTIMATEGENERATORFROMCSV  t,Tm,Te,w CSV'sinden H ve D'yi LS (+ istege bagli PINN) ile bulur.
%   Gercek Simulink kaydi icin simulinkLogToCsv ile CSV uretip bu fonksiyonu cagirin.
%   opts.dampingRef (1: D*(w-1), model varsayimi), opts.pinn (true/false), opts.trueH/trueD
if nargin < 2, opts = struct(); end
if ~isfield(opts, 'dampingRef'), opts.dampingRef = 1; end
if ~isfield(opts, 'pinn'), opts.pinn = false; end
[t, Tm, Te, w] = loadGeneratorCsv(csvPath);
res.ls = estimateHD_LS(t, Tm, Te, w, opts.dampingRef);
res.free = estimateHD_LS(t, Tm, Te, w, NaN);
res.pinn = [];
if opts.pinn, res.pinn = estimateHD_PINN(t, Tm, Te, w, struct('dampingRef', opts.dampingRef)); end
fprintf('%s\n  LS    : H=%.4f  D=%.4f  (wSpan=%.2e, cond=%.1f, identifiable=%d)\n', ...
    csvPath, res.ls.H, res.ls.D, res.ls.wSpan, res.ls.condNum, res.ls.identifiable);
fprintf('  sabit terimli test: c=%.4g  (c~0 -> D*(w-1); c~-D -> D*w)\n', res.free.c);
if opts.pinn, fprintf('  PINN  : H=%.4f  D=%.4f\n', res.pinn.H, res.pinn.D); end
end
