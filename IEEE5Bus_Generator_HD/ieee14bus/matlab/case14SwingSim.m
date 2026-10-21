function out = case14SwingSim(opts)
%CASE14SWINGSIM  IEEE 14 baralı modifiye sistem, klasik makine modeli (Simulink gerektirmez).
%   Simulink'teki ClassicalSM ile aynı denklemler (her makine kendi MVA tabanında):
%       2H*dw/dt = (Pm - Pe)/S - D*(w - 1),   d(delta)/dt = 2*pi*f0*(w - 1)
%   Makineler: ünite 15,16,17,19,18 (ana bara 1,2,3,6,8). Ünite trafoları ideal; ideal
%   kaynak yok (frekansı makineler ve D belirler). Yükler güç akışı geriliminde sabit-Z.
%
%   out = case14SwingSim(opts)  opts (hepsi isteğe bağlı):
%     H, D [5x1] (varsayılan mg14), Tend (10), dt (2e-3), Ts (0.02)
%     pmFun(t)   -> 1x5 Pm çarpanı
%     loadFun(t) -> 14x1 kompleks ek yük [MW+jMVAr] (varsayılan 0); sabit-Z, güç akışı Vm'inde
%     noiseW, noiseP, seed
%   out: t, Tm, Te, w (Nx5, makine bazında pu), H, D, S_MVA, unitBus
if nargin < 1, opts = struct(); end
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, '..', 'mg14_params.mat')); mg = S.mg14;
o.H = mg.gen.H; o.D = mg.gen.D; o.Tend = 10; o.dt = 2e-3; o.Ts = 0.02;
o.pmFun = []; o.loadFun = []; o.noiseW = 0; o.noiseP = 0; o.seed = 1;
fn = fieldnames(opts);
for i = 1:numel(fn), o.(fn{i}) = opts.(fn{i}); end
o.H = o.H(:); o.D = o.D(:);

Sb = mg.base.Sbase_MVA; wb = mg.base.wbase; nb = 14; nm = 5;
Y = zeros(nb);
for k = 1:20
    f = mg.branch.from(k); t2 = mg.branch.to(k); a = mg.branch.tap(k);
    ys = 1/(mg.branch.R_pu(k) + 1i*mg.branch.X_pu(k)); bs = 1i*mg.branch.B_pu(k)/2;
    Y(f,f) = Y(f,f) + (ys + bs)/a^2; Y(t2,t2) = Y(t2,t2) + ys + bs;
    Y(f,t2) = Y(f,t2) - ys/a; Y(t2,f) = Y(t2,f) - ys/a;
end
Vm = mg.bus.Vm_pu(1:nb);
SL0 = mg.bus.Pd_MW(1:nb) + 1i*mg.bus.Qd_MVAr(1:nb) + mg.bus.MGload_MW(1:nb) + 1i*mg.bus.MGload_MVAr(1:nb);
if isempty(o.pmFun), o.pmFun = @(t) ones(1, nm); end
if isempty(o.loadFun), o.loadFun = @(t) zeros(nb, 1); end
sb = mg.gen.sys_bus; Sm = mg.gen.S_MVA;
zg = (mg.gen.ra + 1i*mg.gen.xdp) .* Sb ./ Sm; yg = 1 ./ zg;
Eabs = mg.gen.Eabs_pu; delta0 = mg.gen.delta0_rad;

    function Pe = elecPower(del, t)
        SL = SL0 + o.loadFun(t);
        Ya = Y + diag(conj(SL)/Sb ./ Vm.^2);
        rhs = zeros(nb, 1); Ei = Eabs .* exp(1i*del);
        for kk = 1:nm
            Ya(sb(kk), sb(kk)) = Ya(sb(kk), sb(kk)) + yg(kk);
            rhs(sb(kk)) = rhs(sb(kk)) + yg(kk) * Ei(kk);
        end
        V = Ya \ rhs;
        Pe = zeros(nm, 1);
        for kk = 1:nm
            I = (Ei(kk) - V(sb(kk))) / zg(kk);
            Pe(kk) = real(V(sb(kk)) * conj(I)) * Sb / Sm(kk);
        end
    end

Pm0 = elecPower(delta0, 0);
    function dx = deriv(t, x)
        del = x(1:nm); w = x(nm+1:end);
        Pm = Pm0 .* o.pmFun(t)';
        Pe = elecPower(del, t);
        dx = [wb*(w-1); (Pm - Pe - o.D .* (w-1)) ./ (2*o.H)];
    end

x = [delta0; ones(nm, 1)];
nstep = round(o.Tend/o.dt); nrec = round(o.Ts/o.dt);
N = floor(nstep/nrec) + 1;
T = zeros(N,1); Tm = zeros(N,nm); Te = Tm; W = Tm; Dl = Tm; ir = 0; t = 0;
for s = 0:nstep
    if mod(s, nrec) == 0
        ir = ir + 1; T(ir) = t; W(ir,:) = x(nm+1:end)'; Dl(ir,:) = x(1:nm)';
        Tm(ir,:) = (Pm0 .* o.pmFun(t)')'; Te(ir,:) = elecPower(x(1:nm), t)';
    end
    if s == nstep, break; end
    k1 = deriv(t, x); k2 = deriv(t+o.dt/2, x+o.dt/2*k1);
    k3 = deriv(t+o.dt/2, x+o.dt/2*k2); k4 = deriv(t+o.dt, x+o.dt*k3);
    x = x + o.dt/6*(k1 + 2*k2 + 2*k3 + k4); t = t + o.dt;
end
T = T(1:ir); Tm = Tm(1:ir,:); Te = Te(1:ir,:); W = W(1:ir,:); Dl = Dl(1:ir,:);
if o.noiseW > 0 || o.noiseP > 0
    rng(o.seed, 'twister');
    W = W + o.noiseW*randn(size(W)); Tm = Tm + o.noiseP*randn(size(Tm)); Te = Te + o.noiseP*randn(size(Te));
end
out = struct('t', T, 'Tm', Tm, 'Te', Te, 'w', W, 'delta', Dl, 'H', o.H, 'D', o.D, ...
    'S_MVA', Sm, 'unitBus', mg.gen.unit_bus', 'Pm0_pu', Pm0, 'f0', mg.base.fbase);
end
