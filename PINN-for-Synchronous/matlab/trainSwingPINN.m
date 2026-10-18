function [predH, predD, hist, model] = trainSwingPINN(t, Ydata, opts)
%TRAINSWINGPINN Inverse PINN for the swing equation of the RTDS machine data.
%
%   2*H*dw/dt = Tm - Te - D*(w - dampingRef)
%
% The labeled RTDS traces (1208_*.csv) obey dampingRef = 0, i.e. the damping
% torque acts on the per-unit speed itself (D*w). Least squares on the true
% signals recovers H and D exactly with this form, whereas D*(w-1) leaves a
% residual of about D.
%
% Training: Adam steps ('iterations') first fit the measured curves only
% (warmupFraction = 1). Then full-batch L-BFGS ('lbfgsIterations') minimizes
% data + physics loss and identifies H and D. Pure Adam on data + physics from
% the start converges very slowly and can push H toward 0.
%
% Columns of Ydata: [Tm, w, Te].  hist columns: [iteration, H, D, loss].
if nargin < 3, opts = struct(); end
opts = fillDefault(opts, 'iterations', 1000);
opts = fillDefault(opts, 'lbfgsIterations', 3000);
opts = fillDefault(opts, 'lr', 3e-3);
opts = fillDefault(opts, 'lrParam', 3e-3);
opts = fillDefault(opts, 'lrDecay', 0.1);
opts = fillDefault(opts, 'warmupFraction', 1.0);
opts = fillDefault(opts, 'period', 200);
opts = fillDefault(opts, 'initH', 1.0);
opts = fillDefault(opts, 'initD', 1.0);
opts = fillDefault(opts, 'hidden', 50);
opts = fillDefault(opts, 'dampingRef', 0.0);
opts = fillDefault(opts, 'nFourier', 12);
opts = fillDefault(opts, 'physicsWeight', 1.0);
opts = fillDefault(opts, 'residualScale', 1.0);
opts = fillDefault(opts, 'verbose', true);

t = t(:)';
Yd = Ydata';
N = numel(t);
tn = (t - t(1)) / (t(end) - t(1));

ctx.G = derivativeMatrix(t);
ctx.kLin = Yd(:, 1) + (Yd(:, end) - Yd(:, 1)) * tn;
ctx.fac = 4 * tn .* (1 - tn);
ctx.scale = max(std(Yd, 0, 2), 1e-3);
ctx.X = fourierFeatures(tn, opts.nFourier);
ctx.Yd = Yd;
ctx.N = N;
ctx.sizes = [size(ctx.X, 1), opts.hidden, opts.hidden, opts.hidden, 3];
ctx.dampingRef = opts.dampingRef;
ctx.rs = opts.residualScale;

theta = initTheta(ctx.sizes, opts.initH, opts.initD);
nTheta = numel(theta);
m1 = zeros(nTheta, 1);
v1 = zeros(nTheta, 1);
hist = zeros(0, 4);

nWarm = round(opts.warmupFraction * opts.iterations);
for it = 1:opts.iterations
    ctx.wPhys = opts.physicsWeight * (it > nWarm);
    [L, g] = lossGrad(theta, ctx);
    decay = opts.lrDecay ^ (it / opts.iterations);
    lrVec = opts.lr * decay * ones(nTheta, 1);
    lrVec(end - 1:end) = opts.lrParam * decay;
    m1 = 0.9 * m1 + 0.1 * g;
    v1 = 0.999 * v1 + 0.001 * g .^ 2;
    theta = theta - lrVec .* (m1 / (1 - 0.9 ^ it)) ./ (sqrt(v1 / (1 - 0.999 ^ it)) + 1e-8);
    if it == 1 || mod(it, opts.period) == 0 || it == opts.iterations
        hist(end + 1, :) = [it, theta(end - 1), theta(end), L]; %#ok<AGROW>
        if opts.verbose
            fprintf('  Adam  iter %d  loss %.4e  H=%.4f  D=%.4f\n', it, L, theta(end - 1), theta(end));
        end
    end
end

if opts.lbfgsIterations > 0
    ctx.wPhys = opts.physicsWeight;
    fun = @(th) lossGrad(th, ctx);
    report = @(k, L, th) reportLbfgs(k, L, th, opts, opts.iterations);
    [theta, ~, histL] = lbfgs(fun, theta, opts.lbfgsIterations, report);
    hist = [hist; histL];
end

predH = theta(end - 1);
predD = theta(end);
[~, ~, aux] = lossGrad(theta, setfield(ctx, 'wPhys', opts.physicsWeight)); %#ok<SFLD>
model = struct('theta', theta, 'ctx', ctx, 'Yfit', aux.Y');
end

function histRow = reportLbfgs(k, L, th, opts, offset)
histRow = [offset + k, th(end - 1), th(end), L];
if opts.verbose && (k == 1 || mod(k, 50) == 0)
    fprintf('  L-BFGS iter %d  loss %.4e  H=%.4f  D=%.4f\n', k, L, th(end - 1), th(end));
end
end

function opts = fillDefault(opts, name, value)
if ~isfield(opts, name) || isempty(opts.(name))
    opts.(name) = value;
end
end

function [L, g, aux] = lossGrad(theta, ctx)
[W, b, H, D] = unpackTheta(theta, ctx.sizes);
nL = numel(W);
A = cell(nL + 1, 1);
Z = cell(nL, 1);
S = cell(nL, 1);
A{1} = ctx.X;
for i = 1:nL
    Z{i} = W{i} * A{i} + b{i};
    if i < nL
        S{i} = 1 ./ (1 + exp(-Z{i}));
        A{i + 1} = Z{i} .* S{i};
    else
        A{i + 1} = Z{i};
    end
end
Y = ctx.kLin + ctx.fac .* A{nL + 1};
w = Y(2, :);
dwdt = (ctx.G * w')';
r = Y(1, :) - Y(3, :) - 2 * H * dwdt - D * (w - ctx.dampingRef);

Ldata = mean(mean(((Y - ctx.Yd) ./ ctx.scale) .^ 2));
Lphy = mean(r .^ 2) / ctx.rs ^ 2;
L = Ldata + ctx.wPhys * Lphy;

gY = 2 * (Y - ctx.Yd) ./ (ctx.scale .^ 2) / numel(Y);
gR = ctx.wPhys * 2 * r / ctx.rs ^ 2 / ctx.N;
gY(1, :) = gY(1, :) + gR;
gY(3, :) = gY(3, :) - gR;
gY(2, :) = gY(2, :) - 2 * H * (ctx.G' * gR')' - D * gR;
gH = sum(gR .* (-2 * dwdt));
gD = sum(gR .* (-(w - ctx.dampingRef)));

Gc = gY .* ctx.fac;
gW = cell(nL, 1);
gb = cell(nL, 1);
for i = nL:-1:1
    if i < nL
        Gc = Gc .* (S{i} + Z{i} .* S{i} .* (1 - S{i}));
    end
    gW{i} = Gc * A{i}';
    gb{i} = sum(Gc, 2);
    if i > 1
        Gc = W{i}' * Gc;
    end
end
g = packTheta(gW, gb, gH, gD);
aux.Y = Y;
end

function theta = initTheta(sizes, H, D)
W = cell(numel(sizes) - 1, 1);
b = cell(numel(sizes) - 1, 1);
for i = 1:numel(sizes) - 1
    limit = sqrt(6 / (sizes(i) + sizes(i + 1)));
    W{i} = (2 * rand(sizes(i + 1), sizes(i)) - 1) * limit;
    b{i} = zeros(sizes(i + 1), 1);
end
theta = packTheta(W, b, H, D);
end

function theta = packTheta(W, b, H, D)
parts = cell(1, 2 * numel(W) + 1);
for i = 1:numel(W)
    parts{2 * i - 1} = W{i}(:);
    parts{2 * i} = b{i}(:);
end
parts{end} = [H; D];
theta = vertcat(parts{:});
end

function [W, b, H, D] = unpackTheta(theta, sizes)
nL = numel(sizes) - 1;
W = cell(nL, 1);
b = cell(nL, 1);
pos = 0;
for i = 1:nL
    n = sizes(i + 1) * sizes(i);
    W{i} = reshape(theta(pos + 1:pos + n), sizes(i + 1), sizes(i));
    pos = pos + n;
    b{i} = theta(pos + 1:pos + sizes(i + 1));
    pos = pos + sizes(i + 1);
end
H = theta(pos + 1);
D = theta(pos + 2);
end

function G = derivativeMatrix(t)
% Same stencil as gradient(): central differences inside, one-sided at the ends.
N = numel(t);
i = (2:N - 1)';
dt = t(i + 1) - t(i - 1);
rows = [i; i];
cols = [i - 1; i + 1];
vals = [-1 ./ dt(:); 1 ./ dt(:)];
G = sparse(rows, cols, vals, N, N);
G(1, 1) = -1 / (t(2) - t(1));
G(1, 2) = 1 / (t(2) - t(1));
G(N, N - 1) = -1 / (t(N) - t(N - 1));
G(N, N) = 1 / (t(N) - t(N - 1));
end

function X = fourierFeatures(tn, nF)
% tn in [0,1]. Sine/cosine pairs let a small MLP follow the ~50 Hz ringing.
X = tn;
for k = 1:nF
    X = [X; sin(k * pi * tn); cos(k * pi * tn)]; %#ok<AGROW>
end
end

function [x, f, histRows] = lbfgs(fun, x, maxIter, report)
% Plain full-batch L-BFGS (memory 20) with Armijo backtracking.
mem = 20;
S = zeros(numel(x), 0);
Yv = zeros(numel(x), 0);
[f, g] = fun(x);
histRows = zeros(0, 4);
for k = 1:maxIter
    q = g;
    nS = size(S, 2);
    alpha = zeros(nS, 1);
    rho = zeros(nS, 1);
    for j = nS:-1:1
        rho(j) = 1 / (Yv(:, j)' * S(:, j));
        alpha(j) = rho(j) * (S(:, j)' * q);
        q = q - alpha(j) * Yv(:, j);
    end
    if nS > 0
        gamma = (S(:, end)' * Yv(:, end)) / (Yv(:, end)' * Yv(:, end));
    else
        gamma = 1 / max(norm(g), 1e-12);
    end
    z = gamma * q;
    for j = 1:nS
        beta = rho(j) * (Yv(:, j)' * z);
        z = z + S(:, j) * (alpha(j) - beta);
    end
    d = -z;
    gd = g' * d;
    if gd >= 0
        d = -g;
        gd = -(g' * g);
        S = zeros(numel(x), 0);
        Yv = zeros(numel(x), 0);
    end
    step = 1;
    accepted = false;
    for ls = 1:30
        xn = x + step * d;
        [fn, gn] = fun(xn);
        if isfinite(fn) && fn <= f + 1e-4 * step * gd
            accepted = true;
            break
        end
        step = step / 2;
    end
    if ~accepted
        break
    end
    s = xn - x;
    y = gn - g;
    x = xn; f = fn; g = gn;
    if s' * y > 1e-12
        S = [S, s]; %#ok<AGROW>
        Yv = [Yv, y]; %#ok<AGROW>
        if size(S, 2) > mem
            S(:, 1) = [];
            Yv(:, 1) = [];
        end
    end
    row = report(k, f, x);
    if k == 1 || mod(k, 10) == 0 || k == maxIter
        histRows(end + 1, :) = row; %#ok<AGROW>
    end
end
end
