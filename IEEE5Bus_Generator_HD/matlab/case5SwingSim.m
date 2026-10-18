function out = case5SwingSim(opts)
%CASE5SWINGSIM  IEEE 5-bus (case5) klasik makine modeli, Simulink gerektirmez.
%   build_transmission.m'deki ClassicalSM modelinin ayni denklemlerini kullanir:
%       2H*dw/dt = (Pm - Pe)/S_base - D*(w - 1)         (makine bazinda pu)
%       d(delta)/dt = 2*pi*f0*(w - 1)
%   Makineler Bus 1, 3, 5'te (EMF X' arkasinda), Bus 4 ideal gerilim kaynagidir.
%   Hatlar pi-kesit, yukler sabit empedans. Veriler inputs/mg_params.mat'tan gelir.
%
%   out = case5SwingSim(opts)   opts alanlari (hepsi istege bagli):
%     H [3x1], D [3x1]   makine 1,2,3 = Bus 1,3,5 (varsayilan 5.5 s, 1.5 pu)
%     Tend (20), dt (2e-3), Ts (0.02)   bitis, adim, kayit periyodu [s]
%     pmFun(t)   -> 1x3 Pm carpani      (varsayilan ones)
%     loadFun(t) -> 5x1 kompleks yuk [MW+jMVAr] (varsayilan Bus3 ve Bus4 nominal)
%     pmMode     'equilibrium' (Pm=Pe(0), varsayilan) | 'pg' (mg.gen.Pg_MW)
%     Xp (0.25)  gecici reaktans, makine bazinda pu
%     noiseW, noiseP  olcum gurultusu std (w ve Tm/Te icin, pu)
%     seed (1)
%   out: t, Tm, Te, w (Nx3, makine bazinda pu), H, D, S_MW, busOfMachine, delta
if nargin < 1, opts = struct(); end
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, '..', 'inputs', 'mg_params.mat')); mg = S.mg;

o.H = [5.5; 5.5; 5.5]; o.D = [1.5; 1.5; 1.5];
o.Tend = 20; o.dt = 2e-3; o.Ts = 0.02; o.pmMode = 'equilibrium'; o.Xp = 0.25;
o.noiseW = 0; o.noiseP = 0; o.seed = 1; o.pmFun = []; o.loadFun = [];
fn = fieldnames(opts);
for i = 1:numel(fn), o.(fn{i}) = opts.(fn{i}); end
o.H = o.H(:); o.D = o.D(:);

Sb = mg.base.Sbase_MVA; f0 = mg.base.fbase; wb = 2*pi*f0;
busM = [1 3 5]; nm = 3;
Pg = zeros(nm,1); Qg = zeros(nm,1); Smw = zeros(nm,1);
for k = 1:nm
    idx = mg.gen.bus == busM(k);
    Pg(k) = sum(mg.gen.Pg_MW(idx)); Qg(k) = sum(mg.gen.Qg_MVAr(idx));
    Smw(k) = max(sum(mg.gen.Pmax_MW(idx)), 1.2*Pg(k));
end

nb = 5; Y = zeros(nb);
for k = 1:numel(mg.branch.from)
    f = mg.branch.from(k); t2 = mg.branch.to(k);
    ys = 1/(mg.branch.R_pu(k) + 1i*mg.branch.X_pu(k));
    bs = 1i*mg.branch.B_pu(k)/2;
    Y(f,f) = Y(f,f) + ys + bs; Y(t2,t2) = Y(t2,t2) + ys + bs;
    Y(f,t2) = Y(f,t2) - ys;    Y(t2,f) = Y(t2,f) - ys;
end

Vt = mg.bus.Vm_pu .* exp(1i*mg.bus.Va_rad);
Xs = o.Xp * Sb ./ Smw;
yg = 1 ./ (1i*Xs);
E = zeros(nm,1);
for k = 1:nm
    Spu = (Pg(k) + 1i*Qg(k)) / Smw(k);
    Imach = conj(Spu / Vt(busM(k)));
    Em = Vt(busM(k)) + 1i*o.Xp*Imach;
    E(k) = Em;
end
Eabs = abs(E); delta0 = angle(E);

if isempty(o.pmFun), o.pmFun = @(t) ones(1,nm); end
if isempty(o.loadFun)
    SL0 = zeros(nb,1); SL0(3) = mg.bus.Pd_MW(3) + 1i*mg.bus.Qd_MVAr(3);
    SL0(4) = mg.bus.Pd_MW(4) + 1i*mg.bus.Qd_MVAr(4);
    o.loadFun = @(t) SL0;
end
SL0t = o.loadFun(0);

ug = [1 2 3 5]; gbus = busM;
    function Pe = elecPower(del, t)
        SLt = o.loadFun(t);
        yL = conj(SLt) / Sb ./ (abs(Vt).^2);
        Ya = Y + diag(yL);
        Yu = Ya(ug, ug);
        Ei = Eabs .* exp(1i*del);
        rhs = zeros(4,1);
        for kk = 1:nm
            rhs(ug == gbus(kk)) = Ei(kk) * yg(kk);
            Yu(ug == gbus(kk), ug == gbus(kk)) = Yu(ug == gbus(kk), ug == gbus(kk)) + yg(kk);
        end
        rhs = rhs - Ya(ug, 4) * 1.0;
        Vu = Yu \ rhs;
        Pe = zeros(nm,1);
        for kk = 1:nm
            Vk = Vu(ug == gbus(kk));
            Ik = yg(kk) * (Ei(kk) - Vk);
            Pe(kk) = real(Ei(kk) * conj(Ik)) * Sb / Smw(kk);
        end
    end

Pe0 = elecPower(delta0, 0);
if strcmp(o.pmMode, 'pg'), Pm0 = Pg ./ Smw; else, Pm0 = Pe0; end

x = [delta0; ones(nm,1)];
    function dx = deriv(t, x)
        del = x(1:nm); w = x(nm+1:end);
        Pm = Pm0 .* o.pmFun(t)';
        Pe = elecPower(del, t);
        dx = [wb*(w-1); (Pm - Pe - o.D .* (w-1)) ./ (2*o.H)];
    end

nstep = round(o.Tend/o.dt); nrec = round(o.Ts/o.dt);
N = floor(nstep/nrec) + 1;
T = zeros(N,1); Tm = zeros(N,nm); Te = zeros(N,nm); W = zeros(N,nm); Dl = zeros(N,nm);
ir = 0; t = 0;
for s = 0:nstep
    if mod(s, nrec) == 0
        ir = ir + 1;
        T(ir) = t; W(ir,:) = x(nm+1:end)'; Dl(ir,:) = x(1:nm)';
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
    W = W + o.noiseW * randn(size(W));
    Tm = Tm + o.noiseP * randn(size(Tm));
    Te = Te + o.noiseP * randn(size(Te));
end
out = struct('t', T, 'Tm', Tm, 'Te', Te, 'w', W, 'delta', Dl, 'H', o.H, 'D', o.D, ...
    'S_MW', Smw, 'busOfMachine', busM, 'Pe0', Pe0, 'Pg_pu', Pg./Smw, 'f0', f0);
end
