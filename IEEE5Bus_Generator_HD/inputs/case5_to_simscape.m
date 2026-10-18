%% =========================================================
%  case5_to_simscape.m  —  v2 (MATPOWER-TÜRETİLMİŞ)
%  case5 MATPOWER verisini Simscape Electrical parametrelerine çevirir.
%
%  HIL-Ready Mikro Şebeke Projesi — YTÜ / TÜBİTAK 2209-A
%  Yazar: Samet Erhan Sarı
%
%  v2 DEĞİŞİKLİĞİ (Kural 8 — case5 verisini ASLA elle girme):
%    Önceki sürüm bus/branch/gen verisini elle (hardcoded) içeriyordu
%    ve hatalar barındırıyordu:
%      • Slack bara Bus 1 işaretlenmiş  — gerçeği Bus 4 (type=3)
%      • Bus 3 jeneratör Pmax = 300 MW  — gerçeği 520 MW
%      • Bus 1 tek 210 MW jeneratör     — gerçeği İKİ jeneratör (40 + 170)
%    Bu sürüm TÜM case5 verisini loadcase('case5') + runpf ile üretir.
%
%  NOT — Güç dengesi: Toplam Üretim (Pg ≈ 1005 MW) ile Toplam Yük
%  (Pd = 1000 MW) eşit DEĞİLDİR; aradaki ~5 MW hat kaybıdır ve runpf
%  bunu slack barada (Bus 4) karşılar. Bu fiziksel olarak doğrudur.
%
%  ÇIKIŞ: 'mg' struct'ı + 'mg_params.mat'
%  ÇALIŞTIRMA SIRASI: bu dosya en önce çalışır (bkz. CLAUDE.md RUN ORDER)
%  =========================================================

clear; clc;

fprintf('=== case5 → Simscape Parametre Çıkarımı (v2, MATPOWER-türetilmiş) ===\n\n');

%% ─────────────────────────────────────────────────────────
%  0. MATPOWER'DAN case5 VERİSİNİ YÜKLE + GÜÇ AKIŞI ÇÖZ
%  ─────────────────────────────────────────────────────────
%  Kural 8: case5 verisi ASLA elle girilmez. loadcase + runpf ile
%  üretilir; aksi halde değerler MATPOWER'dan kayar ve proje
%  güvenilirliği (TÜBİTAK) zedelenir.

mpc  = loadcase('case5');
mpc1 = runpf(mpc, mpoption('verbose', 0, 'out.all', 0));

fprintf('✓ case5 yüklendi ve güç akışı çözüldü (runpf).\n\n');

%% ─────────────────────────────────────────────────────────
%  1. TEMEL BÜYÜKLÜKLER (Base Values)
%  ─────────────────────────────────────────────────────────
%  S_base MATPOWER'dan gelir. case5 gerilim seviyesi belirtmez;
%  iletim şebekesi için standart 230 kV (L-L, RMS) varsayılır.
%  Baz empedans: Z_base = V_base^2 / S_base → (230e3)^2/100e6 = 529 Ω

mg.base.Sbase_MVA  = mpc1.baseMVA;        % [MVA] — MATPOWER'dan
mg.base.Vbase_kV   = 230;                 % [kV] hat-hat — varsayım (iletim)
mg.base.fbase      = 50;                  % [Hz] Türkiye/AB şebekesi
mg.base.wbase      = 2*pi*mg.base.fbase;

mg.base.Vbase_V    = mg.base.Vbase_kV * 1e3;
mg.base.Sbase_VA   = mg.base.Sbase_MVA * 1e6;
mg.base.Ibase_A    = mg.base.Sbase_VA / (sqrt(3) * mg.base.Vbase_V);
mg.base.Zbase_ohm  = mg.base.Vbase_V^2 / mg.base.Sbase_VA;

fprintf('Baz Büyüklükler:\n');
fprintf('  S_base = %.0f MVA  (mpc.baseMVA)\n', mg.base.Sbase_MVA);
fprintf('  V_base = %.0f kV (L-L)\n', mg.base.Vbase_kV);
fprintf('  Z_base = %.2f Ω\n', mg.base.Zbase_ohm);
fprintf('  I_base = %.2f A\n\n', mg.base.Ibase_A);

%% ─────────────────────────────────────────────────────────
%  2. BUS VERİSİ — mpc1.bus'tan türetilir
%  ─────────────────────────────────────────────────────────
%  MATPOWER bus matrisi sütunları:
%  [bus_id, type, Pd, Qd, Gs, Bs, area, Vm, Va, baseKV, zone, Vmax, Vmin]
%  type: 1=PQ (yük), 2=PV (gen), 3=slack

mg.bus.id      = mpc1.bus(:,1);
mg.bus.type    = mpc1.bus(:,2);           % 1=PQ, 2=PV, 3=slack
mg.bus.Pd_MW   = mpc1.bus(:,3);           % Aktif yük [MW]
mg.bus.Qd_MVAr = mpc1.bus(:,4);           % Reaktif yük [MVAr]
mg.bus.Vm_pu   = mpc1.bus(:,8);           % Gerilim büyüklüğü [pu] (runpf çözümü)
mg.bus.Va_deg  = mpc1.bus(:,9);           % Gerilim açısı [derece]
n_bus          = numel(mg.bus.id);

% Fiziksel değerler
mg.bus.Vm_V   = mg.bus.Vm_pu  * mg.base.Vbase_V;    % [V, RMS L-L]
mg.bus.Va_rad = mg.bus.Va_deg * pi/180;             % [rad]
mg.bus.Pd_W   = mg.bus.Pd_MW  * 1e6;
mg.bus.Qd_VAR = mg.bus.Qd_MVAr* 1e6;
mg.bus.Sd_VA  = sqrt(mg.bus.Pd_W.^2 + mg.bus.Qd_VAR.^2);
mg.bus.pf     = mg.bus.Pd_W ./ (mg.bus.Sd_VA + eps); % Güç faktörü (yüksüz bara → 0)

% Slack bara — type==3 olan baradan türet (ASLA elle Bus 1 deme)
slack_idx     = find(mg.bus.type == 3);
mg.slack.bus  = mg.bus.id(slack_idx);     % case5 → Bus 4
mg.slack.idx  = slack_idx;

% Mikroşebeke PCC — Bus 2 (proje tasarım kararı, case5 verisi değil)
mg.microgrid.pcc_bus  = 2;
mg.microgrid.Pload_MW = mg.bus.Pd_MW(mg.bus.id == mg.microgrid.pcc_bus);

fprintf('Bus Verileri (runpf çözümü):\n');
fprintf('  %-5s %-7s %-11s %-12s %-10s %-8s\n', ...
    'Bus','Type','Pd [MW]','Qd [MVAr]','|V| [pu]','PF');
type_str = {'PQ','PV','Slack'};
for i = 1:n_bus
    t = min(mg.bus.type(i), 3);
    fprintf('  %-5d %-7s %-11.2f %-12.2f %-10.4f %-8.4f\n', ...
        mg.bus.id(i), type_str{t}, ...
        mg.bus.Pd_MW(i), mg.bus.Qd_MVAr(i), mg.bus.Vm_pu(i), mg.bus.pf(i));
end
fprintf('  → Slack bara: Bus %d   |   PCC (mikroşebeke): Bus %d  (%.0f MW yük)\n\n', ...
    mg.slack.bus, mg.microgrid.pcc_bus, mg.microgrid.Pload_MW);

%% ─────────────────────────────────────────────────────────
%  3. BRANCH VERİSİ → SIMSCAPE EMPEDANS PARAMETRELERİ
%  ─────────────────────────────────────────────────────────
%  MATPOWER branch: [from, to, R(pu), X(pu), B(pu), ...]
%  Per-unit → fiziksel: Z_base ile çarp.  X = ωL,  B = ωC.
%  Simscape'te Three-Phase PI Section Line bloğu R[Ω], L[H], C[F] ister.

mg.branch.from = mpc1.branch(:,1);
mg.branch.to   = mpc1.branch(:,2);
mg.branch.R_pu = mpc1.branch(:,3);
mg.branch.X_pu = mpc1.branch(:,4);
mg.branch.B_pu = mpc1.branch(:,5);
n_branch       = numel(mg.branch.from);

% Per-unit → Ohm / Henry / Farad
mg.branch.R_ohm   = mg.branch.R_pu * mg.base.Zbase_ohm;
mg.branch.X_ohm   = mg.branch.X_pu * mg.base.Zbase_ohm;
mg.branch.B_S     = mg.branch.B_pu / mg.base.Zbase_ohm;   % Susceptans [S]
mg.branch.L_henry = mg.branch.X_ohm / mg.base.wbase;      % X = ωL
mg.branch.C_farad = mg.branch.B_S   / mg.base.wbase;      % B = ωC

% Simscape PI Line — km başına parametreler (uzunluk normalize, 100 km)
LINE_LENGTH_KM = 100;
mg.branch.length_km      = LINE_LENGTH_KM * ones(n_branch,1);
mg.branch.R_ohm_per_km   = mg.branch.R_ohm   / LINE_LENGTH_KM;
mg.branch.L_henry_per_km = mg.branch.L_henry / LINE_LENGTH_KM;
mg.branch.C_farad_per_km = mg.branch.C_farad / LINE_LENGTH_KM;

fprintf('Branch Parametreleri (Fiziksel — Simscape PI Line için):\n');
fprintf('  %-12s %-11s %-11s %-11s\n', 'Hat','R[Ω]','L[mH]','C[µF]');
for i = 1:n_branch
    fprintf('  Bus%d→Bus%-4d %-11.4f %-11.4f %-11.6f\n', ...
        mg.branch.from(i), mg.branch.to(i), ...
        mg.branch.R_ohm(i), mg.branch.L_henry(i)*1e3, mg.branch.C_farad(i)*1e6);
end
fprintf('\n');

%% ─────────────────────────────────────────────────────────
%  4. JENERATÖR VERİSİ — mpc1.gen'den türetilir
%  ─────────────────────────────────────────────────────────
%  MATPOWER gen: [bus, Pg, Qg, Qmax, Qmin, Vg, mBase, status, Pmax, Pmin, ...]
%  case5'te 5 jeneratör vardır (Bus 1'de İKİ tane: 40 MW + 170 MW).
%  Pg değerleri runpf çözümünden gelir; slack (Bus 4) hat kayıplarını
%  karşıladığı için küçük ama sıfır olmayan bir Pg üretir.

mg.gen.bus     = mpc1.gen(:,1);
mg.gen.Pg_MW   = mpc1.gen(:,2);
mg.gen.Qg_MVAr = mpc1.gen(:,3);
mg.gen.Vg_pu   = mpc1.gen(:,6);
mg.gen.Pmax_MW = mpc1.gen(:,9);
mg.gen.Pmin_MW = mpc1.gen(:,10);
n_gen          = numel(mg.gen.bus);

% Fiziksel değerler
mg.gen.Pg_W = mg.gen.Pg_MW * 1e6;
mg.gen.Vg_V = mg.gen.Vg_pu * mg.base.Vbase_V;

fprintf('Jeneratör Verileri (runpf çözümü, %d jeneratör):\n', n_gen);
fprintf('  %-5s %-12s %-12s %-11s %-9s\n', 'Bus','Pg [MW]','Qg [MVAr]','Pmax [MW]','Vg [pu]');
for i = 1:n_gen
    fprintf('  %-5d %-12.4f %-12.4f %-11.0f %-9.4f\n', ...
        mg.gen.bus(i), mg.gen.Pg_MW(i), mg.gen.Qg_MVAr(i), ...
        mg.gen.Pmax_MW(i), mg.gen.Vg_pu(i));
end
fprintf('\n');

%% ─────────────────────────────────────────────────────────
%  5. YENİLENEBİLİR + HİDROJEN SİSTEM PARAMETRELERİ
%  ─────────────────────────────────────────────────────────
%  NOT: Bu bölüm case5 verisi DEĞİL — proje tasarım parametreleridir
%  (Kural 8 yalnızca case5 şebeke verisini kapsar). Bus 2'ye bağlanır.

%  PV Sistem (Bus 2 bağlantısı)
mg.pv.bus         = 2;
mg.pv.Ppeak_MW    = 80;            % Kurulu güç [MW]
mg.pv.Vmpp_V      = 600;           % MPP gerilimi (DC link) [V]
mg.pv.NOCT_C      = 45;            % Nominal çalışma sıcaklığı [°C]
mg.pv.Tcoeff      = -0.004;        % Güç sıcaklık katsayısı [/°C]
mg.pv.inverter_eff= 0.97;          % İnvertör verimi

%  Rüzgar Türbin Sistemi (Bus 2 bağlantısı)
mg.wind.bus       = 2;
mg.wind.Prated_MW = 120;           % Nominal güç [MW]
mg.wind.v_cutin   = 3;             % Devreye girme hızı [m/s]
mg.wind.v_rated   = 12;            % Nominal hız [m/s]
mg.wind.v_cutout  = 25;            % Devre dışı hız [m/s]
mg.wind.cp_max    = 0.48;          % Max güç katsayısı (Betz < 0.593)

%  Batarya ESS (Bus 2 — slack destekli)
mg.ess.bus        = 2;
mg.ess.Emax_MWh   = 200;           % Kapasite [MWh]
mg.ess.Pmax_MW    = 80;            % Maks güç [MW]
mg.ess.SOC_init   = 0.60;          % Başlangıç SOC
mg.ess.SOC_min    = 0.15;
mg.ess.SOC_max    = 0.90;
mg.ess.eta_charge = 0.95;
mg.ess.eta_discharge = 0.95;

%  Hidrojen Sistemi (Bus 2 bağlantısı)
mg.h2.bus         = 2;

%  Elektrolizör (PEM — Power-to-Gas)
mg.h2.elec.Pmin_MW      = 5;       % Minimum çalışma gücü
mg.h2.elec.Pmax_MW      = 40;      % Maksimum çalışma gücü
mg.h2.elec.eta          = 0.70;    % Elektrik→H2 verimi (LHV bazlı)
mg.h2.elec.H2_kgperMWh  = 20;      % ~20 kg H2/MWh (PEM tipik)
mg.h2.elec.tau_s        = 30;      % Dinamik zaman sabiti [s]
%  → Elektrolizör hızlı tepki veremez; 30s zaman sabiti HIL'de önemli

%  H2 Deposu
mg.h2.tank.capacity_kg  = 1000;    % [kg]
mg.h2.tank.SOC_init     = 0.50;    % Başlangıç doluluk oranı (hydrogen_system_params ile tutarlı)
mg.h2.tank.SOC_min      = 0.05;
mg.h2.tank.SOC_max      = 0.95;
mg.h2.tank.Pmax_bar     = 350;     % Max basınç [bar]

%  Yakıt Hücresi (PEMFC)
mg.h2.fc.Pmin_MW        = 2;
mg.h2.fc.Pmax_MW        = 30;
mg.h2.fc.eta            = 0.55;    % H2→Elektrik verimi (LHV)
mg.h2.fc.H2_kgperMWh    = 36;      % ~36 kg H2/MWh tüketim
mg.h2.fc.tau_s          = 5;       % FC daha hızlı tepki verir [s]

fprintf('Yenilenebilir + H2 Sistem Özeti:\n');
fprintf('  PV  (Bus %d): Ppeak = %d MW\n', mg.pv.bus, mg.pv.Ppeak_MW);
fprintf('  Wind(Bus %d): Prated = %d MW\n', mg.wind.bus, mg.wind.Prated_MW);
fprintf('  ESS (Bus %d): E = %d MWh, P = %d MW\n', ...
    mg.ess.bus, mg.ess.Emax_MWh, mg.ess.Pmax_MW);
fprintf('  Elektrolizör: %.0f–%d MW, eta=%.0f%%\n', ...
    mg.h2.elec.Pmin_MW, mg.h2.elec.Pmax_MW, mg.h2.elec.eta*100);
fprintf('  Yakıt Hücresi: %.0f–%d MW, eta=%.0f%%\n', ...
    mg.h2.fc.Pmin_MW, mg.h2.fc.Pmax_MW, mg.h2.fc.eta*100);
fprintf('  H2 Deposu: %d kg\n\n', mg.h2.tank.capacity_kg);

%% ─────────────────────────────────────────────────────────
%  6. HIL SOLVER KONFİGÜRASYONU
%  ─────────────────────────────────────────────────────────
%  OPAL-RT tipik adım süreleri:
%    Güç elektroniği (inverter PWM dahil): 10–50 µs
%    Şebeke dinamiği (temel frekans): 50–200 µs
%    Sistem seviyesi kontrolü: 1 ms
%  3 katmanlı timestep kullanılır:

mg.hil.Ts_power_us   = 50;     % Güç çevrimi adımı [µs] → temel adım
mg.hil.Ts_network_us = 50;     % Şebeke çözümü [µs] — aynı adım
mg.hil.Ts_control_ms = 1;      % EMS/kontrol adımı [ms]
mg.hil.Ts_H2_ms      = 100;    % Hidrojen sistemi (yavaş) [ms]
mg.hil.solver = 'ode4';  % DEAD/HISTORICAL — gerçek solver ode14x (build_microgrid_eelib.m). Bu alan hiçbir set_param'a beslenmiyor, koşuda kullanılmıyor.

fprintf('HIL Solver Konfigürasyonu:\n');
fprintf('  Temel adım (güç/şebeke): %d µs\n', mg.hil.Ts_power_us);
fprintf('  Kontrol katmanı: %d ms\n', mg.hil.Ts_control_ms);
fprintf('  H2 katmanı: %d ms\n', mg.hil.Ts_H2_ms);
fprintf('  Solver: %s (OPAL-RT uyumlu)\n\n', mg.hil.solver);

%% ─────────────────────────────────────────────────────────
%  7. DOĞRULAMA — Güç Dengesi Kontrolü
%  ─────────────────────────────────────────────────────────
%  Üretim = Yük + Hat kayıpları olmalı. case5'te kayıp ≈ %0.5.
%  ÖNEMLİ: Toplam Pg ile Toplam Pd EŞİT DEĞİLDİR; aradaki fark
%  hat kaybıdır ve runpf bunu slack barada karşılar.

total_gen_MW  = sum(mg.gen.Pg_MW);
total_load_MW = sum(mg.bus.Pd_MW);
loss_MW       = total_gen_MW - total_load_MW;

fprintf('Güç Dengesi Kontrolü:\n');
fprintf('  Toplam Üretim (Pg) : %.4f MW\n', total_gen_MW);
fprintf('  Toplam Yük    (Pd) : %.4f MW\n', total_load_MW);
fprintf('  Hat Kayıpları      : %.4f MW  (%.3f%%)\n', ...
    loss_MW, loss_MW/total_gen_MW*100);
if loss_MW > 0 && loss_MW < 0.05*total_gen_MW
    fprintf('  → Güç dengesi tutarlı ✓ (kayıp makul aralıkta)\n\n');
else
    fprintf('  → UYARI: kayıp beklenmedik — runpf çözümünü kontrol et ✗\n\n');
end

re_total_MW = mg.pv.Ppeak_MW + mg.wind.Prated_MW;
fprintf('Yenilenebilir Penetrasyon (Kurulu Kapasite):\n');
fprintf('  PV + Rüzgar = %d MW\n', re_total_MW);
fprintf('  Toplam Yük Oranı = %.1f%%\n\n', re_total_MW/total_load_MW*100);

%% ─────────────────────────────────────────────────────────
%  8. KAYDET + mg STRUCT ÖZETİ
%  ─────────────────────────────────────────────────────────
save('mg_params.mat', 'mg');
fprintf('✓ Parametreler "mg_params.mat" dosyasına kaydedildi.\n\n');

fprintf('mg struct alanları:\n');
top_fields = fieldnames(mg);
for i = 1:numel(top_fields)
    sub = fieldnames(mg.(top_fields{i}));
    fprintf('  mg.%-10s → %s\n', top_fields{i}, strjoin(sub', ', '));
end
fprintf('\n');

fprintf('=== Sonraki Adım ===\n');
fprintf('  → stability_analysis.m    : kararlılık analizi (mg_params.mat kullanır)\n');
fprintf('  → island_mode_ems.m       : ada modu geçiş senaryosu\n');
fprintf('  → build_microgrid_eelib.m : ee_lib tabanlı Simscape model kurucu\n');
