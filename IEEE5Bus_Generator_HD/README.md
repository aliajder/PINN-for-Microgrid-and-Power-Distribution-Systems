# IEEE 5-bus jeneratör H ve D tahmini

Amaç: `microgrid_case5.slx` modelindeki klasik senkron jeneratörlerin (Bus 1, 3, 5) atalet sabiti **H** ve sönüm katsayısı **D** değerlerini, yalnızca o jeneratörün `Tm`, `Te`, `ω` zaman serilerinden bulmak.

## Modelin swing denklemi (girdi dosyalarından doğrulandı)

`inputs/SIMULINK/build_transmission.m` (`swing_eq`) ve `IO_CONTRACTS.md` §3.2:

    2H dω/dt = (Pm - Pe)/S_base - D (ω - 1)        (makine bazında pu)

Modelde tasarım değerleri `H = 5.5 s`, `D = 1.5 pu`, `X' = 0.25 pu`. Sönüm **D·(ω−1)** biçimindedir. Önceki RTDS verisi ise **D·ω** biçimindeydi. Bu yüzden bu klasörde `dampingRef = 1` kullanılır. `estimateHD_LS(..., NaN)` sabit terimli regresyonla biçimi veriden de ayırt eder.

Not: Pm ve Pe model içinde doğrudan güç olarak kullanılır. Bu yüzden `Tm = Pm/S_base` ve `Te = Pe/S_base` alınır, ω'ya bölünmez.

## Önemli sınırlama

Bu ortamda MATLAB/Simulink yok, `.slx` çalıştırılamadı. Veri, `build_transmission.m`'deki denklemlerle yazılmış Simulink'siz bir benzetimden (`matlab/case5SwingSim.m`, Octave'da test edildi) üretildi. Hatlar `mg_params.mat`'tan, makine EMF'leri `build_transmission.m` ile aynı yöntemle hesaplanır, Bus 4 ideal kaynaktır. Gerçek model için `simulinkLogToCsv.m` ile kendi kaydınızı CSV'ye çevirip `estimateGeneratorFromCsv.m` ile tahmin yapın.

## Klasör yapısı

| Yol | İçerik |
|---|---|
| `inputs/` | Kullanılan girdiler: `case5_to_simscape.m`, `mg_params.mat`, `h2_params.mat`, `hydrogen_system_params.m`, `SIMULINK/{build_transmission,build_pcc,wire_microgrid}.m`, `microgrid_case5.slx`, `IO_CONTRACTS.md` |
| `matlab/` | Kodlar (aşağıda) |
| `matlab/tests/` | Bağımsız testler |
| `outputs/` | `generator_HD_estimates.csv`, `test_report.csv`, `data/*.csv`, grafikler |
| `plot_outputs.py` | Grafikleri `outputs/`'a çizer |

`inputs/` içinden silinenler (jeneratör H/D tahmini için gereksiz; git geçmişinde duruyor): PV, rüzgar, yakıt hücresi, elektrolizör, H2 tankı, batarya, EMS, DC bara, VSC, MG AC bara, HIL/SIL/PIL betikleri, `DATA/`, `PARAMS/`, `verification/`, ada testi betikleri. `h2_params.mat` ve `hydrogen_system_params.m` kendi isteğiniz üzerine ve ileride alternatif çalışmalar için tutuldu; jeneratör H/D için kullanılmıyor. `MANIFEST.txt`, `PROJECT_KNOWLEDGE.md`, `README_HIL.md` depoda yoktu.

## Kullanım (MATLAB veya Octave)

```matlab
cd IEEE5Bus_Generator_HD/matlab
setupGeneratorPaths
run_all_tests          % 10 bağımsız test, outputs/test_report.csv
run_generator_study    % outputs/generator_HD_estimates.csv
```

### Gerçek `microgrid_case5.slx` kaydı (yerel MATLAB + Simulink)

Tek betik çözücüyü ayarlar, To Workspace ekler, Bus 3'e yumuşak Pm basamağı koyar, simülasyonu koşar, CSV yazar, LS tahminini basar:

```matlab
cd('...\PINN-for-Microgrid-and-Power-Distribution-Systems\IEEE5Bus_Generator_HD\matlab')
runSimulinkGeneratorRecord
% 10 s, ode14x @ 50 us, Bus3 Pm +%8 (0.1 s rampa), git commit (push yok)

runSimulinkGeneratorRecord(struct('StopTime', 10, 'pmStepFrac', 0.08, ...
    'rampSec', 0.1, 'gitCommit', true, 'gitPush', true))
```

`ode4` kullanmayın. Iraksama olursa `D_pu=30` (o zaman bulunan D tasarım 1.5 değil ~30 olur) veya `pmStepFrac=0.05`, `StopTime=2`.

Zaten `out` varsa simülasyonu tekrar koşmadan:

```matlab
exportLoggedSimulinkOut(out, struct('Sbase', [210 520 600], 'gitPush', true))
```

Çıktı: `outputs/data/Simulink_SM_Bus{1,3,5}.csv` ve `outputs/simulink_HD_estimates.csv`.

Kodlar:

- `case5SwingSim.m`: 5 baralı ağ + 3 klasik makine benzetimi (RK4). Seçenekler: H, D, bozucu (`pmFun`, `loadFun`), gürültü.
- `estimateHD_LS.m`: doğrudan en küçük kareler. Hızlı ve temiz veride en doğru yöntem.
- `estimateHD_PINN.m`: `trainSwingPINN.m` için küçük hız sapmalarına uygun ölçekleme ve otomatik Fourier sayısı.
- `runSimulinkGeneratorRecord.m`: yerel MATLAB'de .slx ayarla + koş + CSV + (isteğe bağlı) git.
- `exportLoggedSimulinkOut.m`: mevcut `out` (Pm_MW_b* / Pe_MW_b* / w_pu_b*) → CSV.
- `exportGeneratorCsv.m`, `loadGeneratorCsv.m`, `simulinkLogToCsv.m`, `estimateGeneratorFromCsv.m`: G/Ç.

## Sonuçlar (`outputs/generator_HD_estimates.csv`, gerçek H=5.5, D=1.5)

| Yöntem | H hatası | D hatası |
|---|---|---|
| LS (9 durum) | %0.6 – %1.0 | %1 – %3.3 |
| PINN (3 durum) | %3 – %3.6 | %20 – %43 (D'yi eksik buluyor) |

PINN, H'yi yaklaşık ama D'yi zayıf buluyor; D terimi (ω−1 ~ 1e-3) küçük olduğundan ağın türev hatalarına duyarlı. Daha fazla L-BFGS adımı yardımcı oluyor, ama yavaş. Bu nedenle LS birincil, PINN karşılaştırma yöntemidir.

## Testler (hepsi geçti, `outputs/test_report.csv`)

1. `test_inputs_consistency`: `mg_params.mat` yapısı, slack = Bus 4, Pe(0) ile Pg uyumu (en fazla %20; Bus 2 MG yükü benzetimde yok).
2. `test_equilibrium`: bozucu yokken ω = 1 kalır.
3. `test_damping_form`: sabit terim ≈ 0 çıkar, yani D·(ω−1). Yanlış D·ω biçimi D'yi 0 bulur.
4. `test_ls_grid`: farklı H ∈ {2, 5.5, 9}, D ∈ {0.5, 1.5, 5} ve her makine için LS.
5. `test_disturbance_types`: Pm basamağı/rampası, Bus 3/5 yük değişimi, Bus 2 (PCC) yükü.
6. `test_noise`: ölçüm gürültüsü; ω gürültüsü 1e-5 ve üstünde D bozulur, 1e-4'te H de bozulur.
7. `test_identifiability`: ω değişmiyorsa tahmin "tanımlanamaz" işaretlenir.
8. `test_der_independence`: Bus 2'deki (mikro şebeke/DER) yük dalgalanması jeneratörün kendi H, D tahminini değiştirmez.
9. `test_csv_roundtrip`: CSV ve Simulink log dönüşümü.
10. `test_pinn`: PINN doğru bölgeye gelir (H %10, D %60 toleransı).

Dikkat: Bus 4 ideal kaynak olduğundan Bus 4'teki yük değişimi makineleri etkilemez; bozucuyu Bus 2, 3 veya 5'e uygulayın.
