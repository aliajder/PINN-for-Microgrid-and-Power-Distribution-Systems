function r = estimateHD_LS(t, Tm, Te, w, dampingRef)
%ESTIMATEHD_LS  Swing denkleminden H ve D'yi dogrudan en kucuk karelerle bulur.
%   2H*dw/dt + D*(w - dampingRef) = Tm - Te
%   dampingRef = 1 (Simulink/ClassicalSM modeli, D*dw_pu) veya 0 (D*w, RTDS verisi).
%   dampingRef = NaN verilirse sabit terimli regresyon yapilir:
%       Tm-Te = 2H*dw/dt + D*(w-1) + c   ->   c ~ 0 ise D*(w-1), c ~ -D ise D*w
%   r: H, D, c, dampingRef, rmsRes, wSpan, condNum, identifiable
t = t(:); Tm = Tm(:); Te = Te(:); w = w(:);
n = numel(t); dw = zeros(n,1);
dw(2:n-1) = (w(3:n) - w(1:n-2)) ./ (t(3:n) - t(1:n-2));
dw(1) = (w(2) - w(1)) / (t(2) - t(1));
dw(n) = (w(n) - w(n-1)) / (t(n) - t(n-1));
y = Tm - Te;
if isnan(dampingRef)
    A = [2*dw, w - 1, ones(n,1)];
else
    A = [2*dw, w - dampingRef];
end
sc = sqrt(sum(A.^2, 1)); An = A ./ sc;
theta = (An \ y) ./ sc';
res = y - A*theta;
r.H = theta(1); r.D = theta(2);
if numel(theta) == 3, r.c = theta(3); else, r.c = 0; end
r.dampingRef = dampingRef; r.rmsRes = sqrt(mean(res.^2));
r.wSpan = max(w) - min(w); r.condNum = cond(An);
r.identifiable = r.wSpan > 1e-4 && r.condNum < 1e3;
end
