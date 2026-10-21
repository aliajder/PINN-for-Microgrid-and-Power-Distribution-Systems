function mg14 = case14_to_simscape(opts)
%CASE14_TO_SIMSCAPE  IEEE 14 baralı MODİFİYE (19 baralı) sistemi Simscape parametrelerine çevirir.
%   case5_to_simscape.m'nin 14 baralı karşılığıdır (Kural 8: veri elle girilmez).
%   MATPOWER case14 + runpf, makine dinamik verileri params/ieee14_dynamic_params.m
%   (Demetriou vd., IEEE Syst. J. 2017, Tablo I-III).
%
%   Modifiye sistem (makale Şekil 5): 19 bara. Yeni bara 15..19, jeneratör/kondansatörleri
%   ideal transformatörle ana baralara bağlar:
%       15(22 kV)->1, 16(13.8 kV)->2, 17(13.8 kV)->3, 19(13.8 kV)->6, 18(13.8 kV)->8
%   Gerilim seviyeleri: Bara 1-5 = 220 kV, Bara 6-14 = 132 kV (makale Şekil 4/5).
%   Mikro şebeke (tüm diğer kaynaklar) Bara 19'a (13.8 kV) bağlanır; PCC trafosu 33/13.8 kV.
%
%   mg14 = case14_to_simscape            % varsayılanlar
%   mg14 = case14_to_simscape(struct('mgLoad_MW', 80, 'mgPF', 1.0, 'pccS_MVA', 150))
%   Çıktı: mg14 struct'ı ve ieee14bus/mg14_params.mat
%
%   NOT (güç dengesi): Toplam yük 259 MW, jeneratör ~272 MW. 5 baralı sistemdeki 300 MW'lık
%   MG yükü bu sistemde TAŞINAMAZ. Varsayılan MG yükü 80 MW'tır (toplam yükün ~%30'u).
%   MG yükü güç akışında Bara 6'ya (= Bara 19, ideal trafo) eklenir. Güç faktörü 1.0:
%   MG'nin VSC'si reaktif gücü yerelde üretir (IO_CONTRACTS: PCC'den "saf P" çekilir);
%   25 MVA'lık Bara 19 kondansatörü MG'nin MVAr'ını taşıyamaz.
if nargin < 1, opts = struct(); end
opts = dfl(opts, 'mgLoad_MW', 80);
opts = dfl(opts, 'mgPF', 1.0);
opts = dfl(opts, 'pccS_MVA', 150);
opts = dfl(opts, 'pccLeak_pu', 0.10);
opts = dfl(opts, 'unitTxS_MVA', 1000);
opts = dfl(opts, 'save', true);

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'params'));
fprintf('=== case14 -> Simscape parametre cikarimi (modifiye 19 bara) ===\n');

mpc = loadcase('case14');
sysBusMG = 6;
mpc0 = mpc;
mgP = opts.mgLoad_MW;  mgQ = mgP * tan(acos(opts.mgPF));
mpc.bus(mpc.bus(:,1) == sysBusMG, 3) = mpc.bus(mpc.bus(:,1) == sysBusMG, 3) + mgP;
mpc.bus(mpc.bus(:,1) == sysBusMG, 4) = mpc.bus(mpc.bus(:,1) == sysBusMG, 4) + mgQ;
mpc1 = runpf(mpc, mpoption('verbose', 0, 'out.all', 0));
assert(mpc1.success == 1, 'runpf yakinsamadi');
fprintf('runpf tamam. MG yuku Bara 19 (=6): %.1f MW, %.1f MVAr\n', mgP, mgQ);

dyn = ieee14_dynamic_params();
Sb = mpc1.baseMVA; f0 = 50;
mg14.base = struct('Sbase_MVA', Sb, 'fbase', f0, 'wbase', 2*pi*f0);

% ---------------- baralar (1..14 ana, 15..19 ünite) ----------------
nb14 = 14; nb = 19;
kV = [220*ones(1,5), 132*ones(1,9)];          % bara 1-5 : 220 kV, 6-14 : 132 kV
unitBus = [dyn.machine.unitBus]; sysBus = [dyn.machine.sysBus];
kVunit = zeros(1, nb); kVunit(unitBus) = [dyn.machine.kV];
mg14.bus.id = (1:nb)';
mg14.bus.isUnit = [zeros(nb14,1); ones(nb-nb14,1)];
mg14.bus.Vbase_kV = [kV, kVunit(nb14+1:nb)]';
mg14.bus.type = [mpc1.bus(:,2); zeros(nb-nb14,1)];
Vm = mpc1.bus(:,8); Va = mpc1.bus(:,9);
Pd = mpc0.bus(:,3); Qd = mpc0.bus(:,4); Bs = mpc1.bus(:,6); Gs = mpc1.bus(:,5);
Vm19 = zeros(nb-nb14,1); Va19 = zeros(nb-nb14,1);
sysOf = zeros(nb,1); sysOf(1:nb14) = 1:nb14; sysOf(unitBus) = sysBus;
mg14.bus.sysBus = sysOf;
mg14.bus.Vm_pu = [Vm; Vm(sysOf(nb14+1:nb))];
mg14.bus.Va_deg = [Va; Va(sysOf(nb14+1:nb))];
mg14.bus.Va_rad = mg14.bus.Va_deg * pi/180;
mg14.bus.Vnom_V = mg14.bus.Vbase_kV * 1e3;
% Sabit-Z yük: net S (şönt dahil), güç akışı geriliminde istenen güç çekilecek şekilde.
Pl = zeros(nb,1); Ql = zeros(nb,1);
Pl(1:nb14) = Pd + Gs .* Vm.^2;
Ql(1:nb14) = Qd - Bs .* Vm.^2;
mg14.bus.Pd_MW = Pl; mg14.bus.Qd_MVAr = Ql;
mg14.bus.Pload_only_MW = [Pd; zeros(nb-nb14,1)];
% MG yükü Bara 6'da; Simulink'te Bara 19'daki PCC üzerinden dışarıdan bağlanır:
mg14.bus.MGload_MW = zeros(nb,1); mg14.bus.MGload_MW(sysBusMG) = mgP;
mg14.bus.MGload_MVAr = zeros(nb,1); mg14.bus.MGload_MVAr(sysBusMG) = mgQ;
mg14.bus.Vload_V = mg14.bus.Vm_pu .* mg14.bus.Vnom_V;
fprintf('Toplam yuk (MG haric): %.1f MW, %.1f MVAr\n', sum(Pd), sum(Qd));

% ---------------- jeneratörler / kondansatörler (5 makine) ----------------
g = mpc1.gen;
for k = 1:5
    m = dyn.machine(k);
    row = find(g(:,1) == m.sysBus, 1);
    Pg = g(row,2); Qg = g(row,3); S = m.S_MVA;
    Vt = Vm(m.sysBus) * exp(1i*Va(m.sysBus)*pi/180);
    Spu = (Pg + 1i*Qg) / S;
    I = conj(Spu / Vt);
    E = Vt + (m.ra + 1i*m.xdp) * I;
    mg14.gen.unit_bus(k,1) = m.unitBus; mg14.gen.sys_bus(k,1) = m.sysBus;
    mg14.gen.operation{k,1} = m.operation;
    mg14.gen.Pg_MW(k,1) = Pg; mg14.gen.Qg_MVAr(k,1) = Qg;
    mg14.gen.S_MVA(k,1) = S; mg14.gen.kV(k,1) = m.kV;
    mg14.gen.H(k,1) = m.H; mg14.gen.D(k,1) = m.D;
    mg14.gen.ra(k,1) = m.ra; mg14.gen.xd(k,1) = m.xd; mg14.gen.xq(k,1) = m.xq;
    mg14.gen.xdp(k,1) = m.xdp; mg14.gen.xqp(k,1) = m.xqp;
    mg14.gen.xdpp(k,1) = m.xdpp; mg14.gen.xl(k,1) = m.xl;
    mg14.gen.Td0p(k,1) = m.Td0p;
    mg14.gen.E_pu(k,1) = E; mg14.gen.Eabs_pu(k,1) = abs(E);
    mg14.gen.delta0_rad(k,1) = angle(E);
    mg14.gen.Vt_pu(k,1) = Vt;
    mg14.gen.Pmax_MW(k,1) = g(row,9);
    fprintf('  unit %2d (bus %2d) %-4s S=%5.0f MVA  Pg=%7.2f MW  Qg=%7.2f MVAr  H=%.3f  D=%.1f  x''d=%.3f\n', ...
        m.unitBus, m.sysBus, m.operation, S, Pg, Qg, m.H, m.D, m.xdp);
end
mg14.gen.Zbase_ohm = (mg14.gen.kV*1e3).^2 ./ (mg14.gen.S_MVA*1e6);
mg14.gen.Xp_ohm = mg14.gen.xdp .* mg14.gen.Zbase_ohm;
mg14.gen.Ra_ohm = mg14.gen.ra  .* mg14.gen.Zbase_ohm;
mg14.gen.Lp_henry = mg14.gen.Xp_ohm / (2*pi*f0);
mg14.gen.Vemf_peak_V = mg14.gen.kV*1e3*sqrt(2/3) .* mg14.gen.Eabs_pu;
mg14.exciter = dyn.exciter; mg14.governor = dyn.governor;

% ---------------- dallar: 20 hat/trafo + 5 ünite trafosu ----------------
br = mpc1.branch; nl = size(br,1);
f = br(:,1); t = br(:,2);
tap = br(:,9); tap(tap == 0) = 1;
isTx = abs(tap - 1) > 1e-9;
mg14.branch.from = [f; unitBus'];
mg14.branch.to   = [t; sysBus'];
mg14.branch.R_pu = [br(:,3); zeros(5,1)];
mg14.branch.X_pu = [br(:,4); zeros(5,1)];
mg14.branch.B_pu = [br(:,5); zeros(5,1)];
mg14.branch.tap  = [tap; ones(5,1)];
mg14.branch.isXfmr = [isTx; ones(5,1)];
mg14.branch.isUnitTx = [zeros(nl,1); ones(5,1)];
kFrom = mg14.bus.Vbase_kV(mg14.branch.from); kTo = mg14.bus.Vbase_kV(mg14.branch.to);
mg14.branch.kV_from = kFrom; mg14.branch.kV_to = kTo;
% Trafo (4-7, 4-9, 5-6): V1 = tap*kV_from, V2 = kV_to (MATPOWER'ın ideal oran modeli)
mg14.branch.V1rated_kV = kFrom .* mg14.branch.tap;
mg14.branch.V2rated_kV = kTo;
Zb = kFrom.^2 / Sb;
mg14.branch.Zbase_ohm = Zb;
mg14.branch.R_ohm = mg14.branch.R_pu .* Zb;
mg14.branch.X_ohm = mg14.branch.X_pu .* Zb;
mg14.branch.L_henry = mg14.branch.X_ohm / (2*pi*f0);
mg14.branch.B_S = mg14.branch.B_pu ./ Zb;
mg14.branch.C_farad = mg14.branch.B_S / (2*pi*f0);
mg14.branch.unitTxS_MVA = opts.unitTxS_MVA;
mg14.branch.unitTxLeak_pu = 1e-4;

% ---------------- mikro şebeke ve PCC ----------------
mg14.microgrid.pcc_bus = 19;
mg14.microgrid.pcc_sys_bus = sysBusMG;
mg14.microgrid.Pload_MW = mgP;
mg14.microgrid.Qload_MVAr = mgQ;
mg14.microgrid.pf = opts.mgPF;
mg14.microgrid.kV_pcc_hv = 13.8;
mg14.microgrid.kV_mv = 33;
mg14.pcc.S_MVA = opts.pccS_MVA;
mg14.pcc.Xleak_pu = opts.pccLeak_pu;
mg14.pcc.V_hv_LL = 13.8e3; mg14.pcc.V_mv_LL = 33e3;
mg14.slack.bus = 1; mg14.slack.note = 'Bara 1 makinesi (ünite 15) güç akışı slack''i; dinamikte gerçek makine, ideal kaynak yok.';
mg14.source = dyn.source;

% ---------------- tutarlılık: makine Pe(0) = Pg ----------------
[Pe, Vcalc] = phasorEquilibrium(mg14);
mg14.gen.Pe0_MW = Pe;
mg14.check.maxPe_err_MW = max(abs(Pe - mg14.gen.Pg_MW));
mg14.check.maxV_err_pu = max(abs(abs(Vcalc(1:nb14)) - Vm));
fprintf('Denge kontrolu: max|Pe0-Pg| = %.4g MW, max|V-Vrunpf| = %.3g pu\n', ...
    mg14.check.maxPe_err_MW, mg14.check.maxV_err_pu);
Pgtot = sum(mg14.gen.Pg_MW);
fprintf('Toplam uretim %.2f MW, toplam yuk (MG dahil) %.2f MW, kayip %.2f MW\n', ...
    Pgtot, sum(Pd) + mgP, Pgtot - sum(Pd) - mgP);

if opts.save
    out = fullfile(here, 'mg14_params.mat');
    save(out, 'mg14', '-v7');
    fprintf('Kaydedildi: %s\n', out);
end
end

function opts = dfl(opts, name, value)
if ~isfield(opts, name) || isempty(opts.(name)), opts.(name) = value; end
end

function [Pe_MW, V] = phasorEquilibrium(mg14)
% Ağın fazör çözümü: EMF'ler sabit, yükler güç akışı geriliminde sabit-Z.
Sb = mg14.base.Sbase_MVA; nb = 14;
Y = zeros(nb);
nl = 20;
for k = 1:nl
    f = mg14.branch.from(k); t = mg14.branch.to(k); a = mg14.branch.tap(k);
    ys = 1/(mg14.branch.R_pu(k) + 1i*mg14.branch.X_pu(k));
    bs = 1i*mg14.branch.B_pu(k)/2;
    Y(f,f) = Y(f,f) + (ys + bs)/a^2; Y(t,t) = Y(t,t) + ys + bs;
    Y(f,t) = Y(f,t) - ys/a; Y(t,f) = Y(t,f) - ys/a;
end
Vm = mg14.bus.Vm_pu(1:nb);
SL = (mg14.bus.Pd_MW(1:nb) + 1i*mg14.bus.Qd_MVAr(1:nb) + ...
     mg14.bus.MGload_MW(1:nb) + 1i*mg14.bus.MGload_MVAr(1:nb));
yL = conj(SL)/Sb ./ (Vm.^2);
Ya = Y + diag(yL);
nm = numel(mg14.gen.sys_bus);
rhs = zeros(nb,1);
for k = 1:nm
    b = mg14.gen.sys_bus(k); S = mg14.gen.S_MVA(k);
    zg = (mg14.gen.ra(k) + 1i*mg14.gen.xdp(k)) * Sb / S;
    yg = 1/zg;
    Ya(b,b) = Ya(b,b) + yg;
    rhs(b) = rhs(b) + yg * mg14.gen.E_pu(k);
end
V = Ya \ rhs;
Pe_MW = zeros(nm,1);
for k = 1:nm
    b = mg14.gen.sys_bus(k); S = mg14.gen.S_MVA(k);
    zg = (mg14.gen.ra(k) + 1i*mg14.gen.xdp(k)) * Sb / S;
    I = (mg14.gen.E_pu(k) - V(b)) / zg;
    Pe_MW(k) = real(V(b) * conj(I)) * Sb;   % terminal gücü (Simulink Pe_calc ile aynı)
end
end
