function r = estimateHD_PINN(t, Tm, Te, w, opts)
%ESTIMATEHD_PINN  Simulink-tipi (D*(w-1)) jenerator verisinden H ve D'yi PINN ile bulur.
%   Kucuk hiz sapmalari (|w-1| ~ 1e-4..1e-3) icin veri su sekilde olceklenir:
%       wt = 1 + k*(w-1),  Tt = k*(Tm-Te)   =>   Tt = 2H*dwt/dt + D*(wt-1)
%   Bu donusum H ve D'yi degistirmez, yalnizca agin sayisal olarak iyi kosullu
%   olmasini saglar. nFourier verilmezse baskin salinim frekansindan secilir. opts: trainSwingPINN secenekleri (dampingRef=1 varsayilan).
if nargin < 5, opts = struct(); end
if ~isfield(opts, 'dampingRef'), opts.dampingRef = 1; end
if ~isfield(opts, 'iterations'), opts.iterations = 500; end
if ~isfield(opts, 'lbfgsIterations'), opts.lbfgsIterations = 8000; end
if ~isfield(opts, 'period'), opts.period = 100; end
if ~isfield(opts, 'verbose'), opts.verbose = false; end
if ~isfield(opts, 'initH'), opts.initH = 1; end
if ~isfield(opts, 'initD'), opts.initD = 1; end
if ~isfield(opts, 'stride'), opts.stride = 2; end
if ~isfield(opts, 'nFourier')
    Ts = mean(diff(t)); wc = w(:) - mean(w(:)); Xf = abs(fft(wc));
    fv = (0:numel(wc)-1)' / (numel(wc)*Ts); [~, im] = max(Xf(2:floor(end/2)));
    opts.nFourier = min(100, max(12, ceil(4 * fv(im+1) * (t(end) - t(1)))));
end
idx = 1:opts.stride:numel(t);
t = t(idx); Tm = Tm(idx); Te = Te(idx); w = w(idx);
kappa = 1 / max(std(w), 1e-9);
wt = 1 + kappa*(w(:) - 1);
Tmt = kappa*Tm(:); Tet = kappa*Te(:);
opts.residualScale = max(std(Tmt - Tet), 1e-9);
[H, D, hist, model] = trainSwingPINN(t, [Tmt, wt, Tet], opts);
r.H = H; r.D = D; r.hist = hist; r.model = model; r.kappa = kappa;
end
