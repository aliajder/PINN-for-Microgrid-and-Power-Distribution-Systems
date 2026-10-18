%% =========================================================
%  hydrogen_system_params.m
%  P2G + H2 Depo + Yakıt Hücresi dinamik modeli
%  Simscape bloklarına doğrudan bağlanacak parametreler
%
%  Model Yaklaşımı:
%  PEM elektrolizör ve PEMFC için ortalama (averaged) modeller
%  kullanıyoruz — PWM switchingleri değil, eşdeğer devresel model.
%  Bu HIL için ideal: yüksek frekanslı anahtarlama gürültüsü olmadan
%  sistem dinamiklerini doğru yakalarız.
%  =========================================================

%% ─────────────────────────────────────────────────────────
%  PEM ELEKTROLİZÖR MODELİ
%  ─────────────────────────────────────────────────────────
%
%  Kimyasal tepkime: 2H2O → 2H2 + O2
%
%  Polarizasyon eğrisi — ULLEBERG (2003) ampirik modeli (build_electrolyzer.m):
%  U_cell = U_rev + (r1+r2·T)·i + s·log10((t1+t2/T+t3/T²)·i + 1)
%
%  U_rev            = tersinir hücre gerilimi ≈ 1.229 V (termodinamik eşik)
%  (r1+r2·T)·i      = ohmik aşırı-gerilim (membran + elektrot iR)
%  s·log10(...)     = aktivasyon aşırı-gerilimi (Tafel)
%  i = I/A [A/m²], T = 80 °C sabit (ısıl model yok, R5)
%
%  Simscape'te (averaged): tek eşdeğer yığın → ONE Controlled Current Source,
%  P→I tersine çözüm (bisection), H2 = AKIM-tabanlı Faraday yasası. PWM yok.

elec = struct();

% Hücre karakteristikleri (PEM, tipik 1 MW modül)
elec.n_cells    = 400;      % Seri hücre sayısı
elec.A_cm2      = 300;      % Aktif membran alanı [cm²]
elec.T_C        = 80;       % Çalışma sıcaklığı [°C]
elec.T_K        = elec.T_C + 273.15;

% Elektrokimyasal sabitler
elec.F          = 96485;    % Faraday sabiti [C/mol]
elec.R_gas      = 8.314;    % İdeal gaz sabiti [J/mol/K]
elec.n_e        = 2;        % H2 başına transfer edilen elektron sayısı

% Tersinir gerilim (Nernst denklemi, basitleştirilmiş)
elec.U_rev      = 1.229;    % [V] standart koşulda
% Sıcaklık düzeltmesi: U_rev(T) ≈ 1.229 - 0.9e-3*(T-298)
elec.U_rev_T    = elec.U_rev - 0.9e-3*(elec.T_K - 298);

% ── ULLEBERG (2003) POLARİZASYON KATSAYILARI (build_electrolyzer.m'de kullanılan)
%   U_cell = U_rev + (r1+r2·T)·i + s·log10((t1+t2/T+t3/T²)·i + 1)   [i=I/A, A/m²]
%   NOT: bu katsayılar build_electrolyzer.m içinde LİTERAL gömülü; burada Rule-8
%   tek-kaynak referansı olarak tutuluyor — değiştirirsen modülü de güncelle.
elec.ull.r1     = 8.05e-5;  % ohmik   [Ω·m²]
elec.ull.r2     = -2.5e-7;  % ohmik   [Ω·m²/°C]
elec.ull.s      = 0.185;    % aktivasyon eğimi [V]
elec.ull.t1     = -0.1002;  % aktivasyon [m²/A]
elec.ull.t2     = 8.424;    % aktivasyon [m²·°C/A]
elec.ull.t3     = 247.3;    % aktivasyon [m²·°C²/A]
elec.ull.A_m2   = 15;       % toplam (aggregate) elektrot alanı [m²]  % TUNED → ~%70 verim
elec.eta_F      = 0.99;     % Faraday verimi (akım→H2)
% (Seri hücre sayısı = elec.n_cells = 400, yukarıda tanımlı.)

% ── DEPRECATED (KULLANILMIYOR): eski Butler-Volmer linearizasyonu. Ulleberg
%   eğrisi (yukarıda) bunların yerini aldı; hiçbir modül bu alanları OKUMUYOR.
%   Geriye dönük uyumluluk için bırakıldı, model davranışına ETKİSİ YOK.
elec.alpha      = 0.5;      % DEPRECATED — yük aktarım katsayısı
elec.i0_Acm2    = 1e-4;     % DEPRECATED — takas akım yoğunluğu [A/cm²]
elec.R_mem_ohm_cm2 = 0.18;  % DEPRECATED — membran özgül direnci [Ω·cm²]
elec.R_mem_ohm = elec.R_mem_ohm_cm2 / elec.A_cm2;  % DEPRECATED — tek hücre [Ω]

% Sistem seviyesi (40 MW modül)
elec.P_rated_MW = 40;
elec.eta_sys    = 0.70;     % Sistem verimi (BOP kayıpları dahil), LHV bazlı
% H2 LHV = 33.3 kWh/kg → 1 MWh elektrik × 0.70 / 33.3 = 21.0 kg/MWh.
% ⚠ 21 kg/MWh ZATEN η=0.70'i içerir. build_electrolyzer.m H2'yi AKIM-tabanlı
%   Faraday ile hesaplar (ṁ = η_F·N·I/(2F)·M_H2) → ~21 kg/MWh DOĞAL olarak çıkar.
%   "P×21/3600×η" gibi tekrar η ile çarpmak ÇİFT-SAYMA olur (eski hata, düzeltildi).
elec.h2_rate_kg_per_MWh = 1000 * elec.eta_sys / 33.3;   % = 21.0 kg/MWh (çapraz-kontrol)

% Dinamik model: 1. dereceden transfer fonksiyonu
% G_elec(s) = 1 / (tau_s * s + 1)
elec.tau_s      = 30;       % [s] — ısıl ve kütlesel ataleti temsil eder
% Bu elektrolizörün hızlı frekans regulasyonu için uygun olmadığını gösterir!

fprintf('Elektrolizör Modeli:\n');
fprintf('  U_rev(%.0f°C) = %.4f V/hücre\n', elec.T_C, elec.U_rev_T);
fprintf('  Sistem verimi: %.0f%%\n', elec.eta_sys*100);
fprintf('  H2 üretim hızı: %.1f kg/MWh\n', elec.h2_rate_kg_per_MWh);
fprintf('  Dinamik zaman sabiti: %d s\n\n', elec.tau_s);

%% ─────────────────────────────────────────────────────────
%  PEM YAKIT HÜCRESİ MODELİ
%  ─────────────────────────────────────────────────────────
%
%  Tepkime: H2 + ½O2 → H2O + Elektrik + Isı
%
%  Polarizasyon eğrisi — CORREA (2004) modeli (build_fc.m):
%  V_cell = E_nernst − V_act − V_ohm − V_con   (akımla DÜŞER — elektrolizörün tersi)
%    V_act = aktivasyon (ξ1..ξ4 Tafel),  V_ohm = i·(R_M+R_C),
%    V_con = −B·ln(1−J/J_max)  (J_max yakınında gaz açlığı)
%  T = 75 °C sabit (R5), p_H2 = 1.5, p_O2 = 1.0 atm.
%
%  Simscape'te (averaged): tek eşdeğer yığın → ONE Controlled Current Source,
%  P→J tersine çözüm (yükselen kol; P(J) tek-tepeli), H2 = AKIM-tabanlı Faraday tüketimi.

fc = struct();

% Hücre karakteristikleri
fc.n_cells      = 600;
fc.A_cm2        = 500;
fc.T_C          = 75;
fc.T_K          = fc.T_C + 273.15;

% Nernst gerilimi
fc.U_nernst     = 1.229;    % [V] standart
fc.U_nernst_T   = fc.U_nernst - 0.9e-3*(fc.T_K - 298);

% Kayıplar
fc.R_int_ohm    = 0.12 / fc.A_cm2;  % İç direnç [Ω] = 2.4e-4 (Correa R_M+R_C kaynağı)
fc.i_L_Acm2     = 1.5;              % Sınır akım yoğunluğu J_max [A/cm²] (V_con)
fc.i0_Acm2      = 2e-4;             % DEPRECATED — kullanılmıyor (Correa kendi ξ'lerini kullanır)

% ── CORREA (2004) POLARİZASYON KATSAYILARI (build_fc.m'de kullanılan)
%   NOT: build_fc.m içinde LİTERAL gömülü; burada Rule-8 tek-kaynak referansı —
%   değiştirirsen modülü de güncelle. (ξ2 modül içinde A ve C_H2'den hesaplanır.)
fc.correa.xi1   = -0.948;   % aktivasyon ξ1 [V]
fc.correa.xi3   = 7.6e-5;   % aktivasyon ξ3
fc.correa.xi4   = -1.93e-4; % aktivasyon ξ4
fc.correa.B     = 0.016;    % konsantrasyon katsayısı [V]
fc.correa.R_M   = 1.9e-4;   % membran direnci [Ω]
fc.correa.R_C   = 5e-5;     % temas direnci [Ω]   (R_M+R_C = R_int_ohm = 2.4e-4)
fc.correa.N_p   = 408;      % paralel dizi sayısı (aggregate) % TUNED → 30 MW @ V_cell≈0.70
fc.correa.p_H2  = 1.5;      % H2 kısmi basıncı [atm]
fc.correa.p_O2  = 1.0;      % O2 kısmi basıncı [atm]
fc.eta_util     = 0.98;     % H2 yakıt kullanım verimi (Faraday tüketim)

% Sistem seviyesi
fc.P_rated_MW   = 30;
fc.eta_sys      = 0.55;     % H2→Elektrik (BOP dahil), LHV bazlı
% H2 LHV = 33.3 kWh/kg → 1 MWh için: 1000/(0.55×33.3) = 54.6 kg/MWh.
% ⚠ 54.6 ZATEN 1/η'yi içerir. build_fc.m tüketimi AKIM-tabanlı Faraday ile
%   hesaplar (ṁ = (P_stack/V_cell)·M_H2/(2F·η_util)) → ~54.6 kg/MWh DOĞAL çıkar.
%   "P×54.6/3600/η" gibi tekrar η'ye bölmek ÇİFT-SAYMA olur (eski hata, düzeltildi).
fc.h2_consume_kg_per_MWh = 1000 / (fc.eta_sys * 33.3);   % = 54.6 kg/MWh (çapraz-kontrol)

% Dinamik model
fc.tau_s        = 5;        % [s] — elektrolizörden daha hızlı
% FC'ler elektrolizörlere göre çok daha hızlı tepki verir
% Bu neden önemli? Frekans regulasyonu için FC kullanabiliriz,
% ama elektrolizörü peak shaving için daha uygun.

fprintf('Yakıt Hücresi Modeli:\n');
fprintf('  U_nernst(%.0f°C) = %.4f V/hücre\n', fc.T_C, fc.U_nernst_T);
fprintf('  Sistem verimi: %.0f%%\n', fc.eta_sys*100);
fprintf('  H2 tüketim hızı: %.1f kg/MWh\n', fc.h2_consume_kg_per_MWh);
fprintf('  Dinamik zaman sabiti: %d s\n\n', fc.tau_s);

%% ─────────────────────────────────────────────────────────
%  H2 DEPO MODELİ
%  ─────────────────────────────────────────────────────────
%
%  Basit kütle dengesi (termodinamik etkiler ihmal):
%  dm_H2/dt = m_dot_in - m_dot_out
%
%  SOC_H2 = m_H2(t) / m_H2_max
%
%  Simscape'te: Integrator blok + SOC hesaplama

tank = struct();
tank.capacity_kg    = 1000;     % [kg]
tank.P_max_bar      = 350;      % [bar]
tank.T_C            = 25;       % Ortam sıcaklığı
tank.SOC_init       = 0.50;     % Başlangıç doluluk — yarı dolu (build_h2tank.m m0=500 kg ile uyumlu)
tank.m_init_kg      = tank.capacity_kg * tank.SOC_init;  % 500 kg başlangıç
tank.SOC_min        = 0.05;     % alt sınır (Rule 7) → 50 kg
tank.SOC_max        = 0.95;     % üst sınır (Rule 7) → 950 kg

% Basınç-kütle ilişkisi — GERÇEK GAZ (Z sıkıştırılabilirlik faktörü):
%   P = Z · m · R · T / (M · V)
% H2 350 bar'da idealden uzaktır: Z ≈ 1.2 (küçük molekül, itme/co-volume rejimi),
% yani aynı kütle için GERÇEK basınç ideal-gazın ~%20 ÜSTÜNDE. İdeal gaz kullanmak
% basıncı düşük gösterir (veya ölçülen 350 bar'ın temsil ettiği kütleyi ~%20 fazla).
tank.M_H2           = 2.016e-3; % H2 molar kütlesi [kg/mol]
tank.R_gas          = 8.314;    % [J/mol/K]
tank.Z              = 1.2;      % H2 sıkıştırılabilirlik faktörü @350 bar (NIST-uyumlu)
% V_tank TÜRETİLDİ: dolu tank (1000 kg) tam olarak 350 bar okusun diye:
%   V = Z·m_cap·R·T / (M·P_max)  ≈ 42.16 m³  →  yoğunluk 23.7 kg/m³ (NIST H2, 350 bar/25°C)
% ⚠ Eski 10 m³ FİZİKSEL HATA idi: 1000 kg/10 m³ = 100 kg/m³; ideal gazla 1000 kg
%   bu hacimde ~1230 bar verirdi (350'nin ~3.5 katı). build_h2tank.m bu formülü
%   kendi içinde aynen türetir → param ile modül artık tutarlı.
tank.V_tank_m3      = tank.Z * tank.capacity_kg * tank.R_gas * (tank.T_C + 273.15) ...
                      / (tank.M_H2 * tank.P_max_bar * 1e5);   % ≈ 42.16 m³

fprintf('H2 Deposu:\n');
fprintf('  Kapasite: %d kg (%.0f bar, %.2f m³, Z=%.2f)\n', ...
    tank.capacity_kg, tank.P_max_bar, tank.V_tank_m3, tank.Z);
fprintf('  Yoğunluk @dolu: %.1f kg/m³ (NIST H2 @350 bar/25°C ≈ 23.7)\n', ...
    tank.capacity_kg / tank.V_tank_m3);
fprintf('  Başlangıç kütlesi: %.0f kg (SOC=%.0f%%), SOC∈[%.2f,%.2f]\n\n', ...
    tank.m_init_kg, tank.SOC_init*100, tank.SOC_min, tank.SOC_max);

%% ─────────────────────────────────────────────────────────
%  ENERJİ YÖNETİM SİSTEMİ (EMS) — H2 Karar Mantığı
%  ─────────────────────────────────────────────────────────
%
%  Karar değişkenleri:
%    P_surplus(t) = P_gen(t) - P_load(t)
%    SOC_ESS(t)   = batarya doluluk oranı
%    SOC_H2(t)    = H2 depo doluluk oranı
%
%  Elektrolizör Devreye Girme Koşulları:
%    P_surplus > P_elec_min  AND  SOC_ESS > 0.80  AND  SOC_H2 < 0.90
%    (Batarya dolu, fazla güç var, depo dolmamış)
%
%  Yakıt Hücresi Devreye Girme Koşulları:
%    P_deficit > P_fc_min  AND  SOC_ESS < 0.20  AND  SOC_H2 > 0.10
%    (Batarya boş, güç açığı var, H2 mevcut)
%
%  Bu mantık Simulink'te Stateflow veya MATLAB Function bloğu ile uygulanır.

ems = struct();
ems.elec_on_SOC_ESS  = 0.80;   % ESS bu seviyenin üstündeyse elektrolizörü çalıştır
ems.elec_off_SOC_H2  = 0.90;   % H2 depo bu seviyeye gelince elektrolizörü durdur
ems.fc_on_SOC_ESS    = 0.20;   % ESS bu seviyenin altındaysa FC'yi çalıştır
ems.fc_off_SOC_H2    = 0.10;   % H2 bu seviyenin altına inince FC'yi durdur

% Frekans kontrolü eşikleri (HIL'de kritik)
ems.f_deadband_Hz    = 0.05;   % ±0.05 Hz deadband
ems.f_droop_percent  = 5;      % %5 droop ayarı
ems.f_trip_Hz        = 0.5;    % ±0.5 Hz → acil müdahale

fprintf('EMS Karar Eşikleri:\n');
fprintf('  Elektrolizör: SOC_ESS > %.0f%%, SOC_H2 < %.0f%%\n', ...
    ems.elec_on_SOC_ESS*100, ems.elec_off_SOC_H2*100);
fprintf('  Yakıt Hücresi: SOC_ESS < %.0f%%, SOC_H2 > %.0f%%\n', ...
    ems.fc_on_SOC_ESS*100, ems.fc_off_SOC_H2*100);
fprintf('  Frekans deadband: ±%.2f Hz\n', ems.f_deadband_Hz);

%% Kaydet
save('h2_params.mat', 'elec', 'fc', 'tank', 'ems');
fprintf('\n✓ H2 parametreleri "h2_params.mat" dosyasına kaydedildi.\n');
