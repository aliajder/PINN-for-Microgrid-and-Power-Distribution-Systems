function [predH, predD, hist] = trainSwingPINN(t, Ydata, opts)
%TRAINSWINGPINN Inverse PINN for Tm-Te-2*H*dw/dt-D*(w-1)=0.
if nargin < 3, opts = struct(); end
opts = fillDefault(opts, 'iterations', 1000);
opts = fillDefault(opts, 'lr', 1e-3);
opts = fillDefault(opts, 'period', 200);
opts = fillDefault(opts, 'initH', 1.0);
opts = fillDefault(opts, 'initD', 1.0);
opts = fillDefault(opts, 'hidden', 50);

t = t(:)';
Yd = Ydata';
N = numel(t);
nSamp = size(Yd, 2);
idx = nSamp - 1;
kLin = (Yd(:, end) - Yd(:, 1)) / (t(end) - t(1));
bLin = (t(end) * Yd(:, 1) - t(1) * Yd(:, end)) / (t(end) - t(1));

net = initNet([6, opts.hidden, opts.hidden, opts.hidden, 3]);
H = opts.initH;
D = opts.initD;
state = initAdam(net, H, D);
hist = zeros(0, 3);

for it = 1:opts.iterations
    [X, ~] = featureTransform(t);
    [Ynn, cache] = forwardNet(net, X);
    fac = tanh(t) .* tanh(idx - t);
    Y = kLin * t + bLin + Ynn .* fac;
    w = Y(2, :);
    dwdt = gradient(w, t);
    r = Y(1, :) - Y(3, :) - 2 * H * dwdt - D * (w - 1);
    Ldata = mean((Y(:) - Yd(:)) .^ 2);
    Lic = mean((Y(:, 1) - Yd(:, 1)) .^ 2);
    Lphy = mean(r .^ 2);
    L = Lphy + Ldata + Lic;

    gY = 2 * (Y - Yd) / numel(Y);
    gY(:, 1) = gY(:, 1) + 2 * (Y(:, 1) - Yd(:, 1)) / 3;
    gR = 2 * r / N;
    gY(1, :) = gY(1, :) + gR;
    gY(3, :) = gY(3, :) - gR;
    gY(2, :) = gY(2, :) - D * gR;
    gH = mean(2 * r .* (-2 * dwdt));
    gD = mean(2 * r .* (-(w - 1)));

    gYnn = gY .* fac;
    gNet = backwardNet(net, cache, gYnn);
    [net, H, D, state] = adamStep(net, H, D, gNet, gH, gD, state, opts.lr, it);

    if it == 1 || mod(it, opts.period) == 0 || it == opts.iterations
        hist(end + 1, :) = [it, H, D]; %#ok<AGROW>
        fprintf('  iter %d  loss %.4e  H=%.4f  D=%.4f\n', it, L, H, D);
    end
end
predH = H;
predD = D;
end

function opts = fillDefault(opts, name, value)
if ~isfield(opts, name) || isempty(opts.(name))
    opts.(name) = value;
end
end

function [X, dXdt] = featureTransform(t)
ts = 0.01 * t;
X = [ts; sin(ts); sin(2 * ts); sin(3 * ts); sin(4 * ts); sin(5 * ts)];
dXdt = 0.01 * [ones(size(ts)); cos(ts); 2 * cos(2 * ts); 3 * cos(3 * ts); 4 * cos(4 * ts); 5 * cos(5 * ts)];
end

function net = initNet(sizes)
net = struct('W', {{}}, 'b', {{}});
for i = 1:numel(sizes) - 1
    fanIn = sizes(i);
    fanOut = sizes(i + 1);
    limit = sqrt(6 / (fanIn + fanOut));
    net.W{i} = (2 * rand(fanOut, fanIn) - 1) * limit;
    net.b{i} = zeros(fanOut, 1);
end
end

function [Y, cache] = forwardNet(net, X)
A = X;
cache.A = {X};
cache.Z = {};
cache.sig = {};
for i = 1:numel(net.W)
    Z = net.W{i} * A + net.b{i};
    cache.Z{i} = Z;
    if i < numel(net.W)
        sig = 1 ./ (1 + exp(-Z));
        A = Z .* sig;
        cache.sig{i} = sig;
    else
        A = Z;
        cache.sig{i} = [];
    end
    cache.A{i + 1} = A;
end
Y = A;
end

function gNet = backwardNet(net, cache, gY)
gNet.W = cell(size(net.W));
gNet.b = cell(size(net.b));
G = gY;
for i = numel(net.W):-1:1
    if i < numel(net.W)
        Z = cache.Z{i};
        sig = cache.sig{i};
        dsw = sig + Z .* sig .* (1 - sig);
        G = G .* dsw;
    end
    gNet.W{i} = G * cache.A{i}';
    gNet.b{i} = sum(G, 2);
    if i > 1
        G = net.W{i}' * G;
    end
end
end

function state = initAdam(net, H, D)
state.mW = cell(size(net.W));
state.vW = cell(size(net.W));
state.mb = cell(size(net.b));
state.vb = cell(size(net.b));
for i = 1:numel(net.W)
    state.mW{i} = zeros(size(net.W{i}));
    state.vW{i} = zeros(size(net.W{i}));
    state.mb{i} = zeros(size(net.b{i}));
    state.vb{i} = zeros(size(net.b{i}));
end
state.mH = 0; state.vH = 0;
state.mD = 0; state.vD = 0;
end

function [net, H, D, state] = adamStep(net, H, D, gNet, gH, gD, state, lr, tstep)
b1 = 0.9; b2 = 0.999; eps = 1e-8;
for i = 1:numel(net.W)
    [net.W{i}, state.mW{i}, state.vW{i}] = adamOne(net.W{i}, gNet.W{i}, state.mW{i}, state.vW{i}, lr, tstep, b1, b2, eps);
    [net.b{i}, state.mb{i}, state.vb{i}] = adamOne(net.b{i}, gNet.b{i}, state.mb{i}, state.vb{i}, lr, tstep, b1, b2, eps);
end
[H, state.mH, state.vH] = adamOne(H, gH, state.mH, state.vH, lr, tstep, b1, b2, eps);
[D, state.mD, state.vD] = adamOne(D, gD, state.mD, state.vD, lr, tstep, b1, b2, eps);
end

function [theta, m, v] = adamOne(theta, g, m, v, lr, tstep, b1, b2, eps)
m = b1 * m + (1 - b1) * g;
v = b2 * v + (1 - b2) * (g .^ 2);
mhat = m / (1 - b1 ^ tstep);
vhat = v / (1 - b2 ^ tstep);
theta = theta - lr * mhat ./ (sqrt(vhat) + eps);
end
