# IEEE 14 bara (modifiye, 19 bara) — dinamik parametreli TransmissionNetwork

Kaynak: `Dynamic_IEEE_Test_Systems_for_Transient_Analysis.pdf` (Demetriou vd., 2017).

## Bara eşlemesi
Makine (ünite) baraları 15-19, sistem baraları 1-14. Ünite trafosu ile bağlanır:
15→1 (22 kV), 16→2, 17→3, 19→6, 18→8 (13.8 kV). Bara 1-5: 220 kV, 6-14: 132 kV.

## Mikro şebeke
Tüm kaynaklar (VSC, DC bara, PV, rüzgar, ESS, EMS, H2) **Bara 19 (13.8 kV)**'a bağlanır:
`Bara 19 — PCC trafosu 13.8/33 kV (Δ-Yg, 150 MVA, %10) — 33 kV MicrogridACBus`.
Bara 19'da ayrıca ünite 19 (25 MVA kondansatör) vardır.

**Uyarı:** sistem üretim kapasitesi ~272 MW; eski 300 MW mikro şebeke yükü mümkün değil.
Varsayılan `Src_Pload_MW = 80`, pf 1.0 (`case14_to_simscape` opsiyonu `mgLoad_MW`).

## Klasörler
- `params/ieee14_dynamic_params.m` — Tablo I-III (H, D, ra, x'd, ... GENROU/IEEET1/BPA_GG)
- `case14_to_simscape.m` — MATPOWER `case14` + yük akışı -> `mg14_params.mat`
- `SIMULINK/build_transmission14.m`, `rebuild_microgrid_case14.m` — .slx üretimi (orijinal .slx değişmez)
- `matlab/` — Simulink'siz RK4 simülatör (`case14SwingSim`), testler, `runSimulinkGeneratorRecord14`
- `outputs/` — Octave çalışma sonuçları (LS: H ≤%3.9, D ≤%1 hata)

## Kullanım (MATLAB, MATPOWER yolda)
```matlab
cd IEEE5Bus_Generator_HD/ieee14bus
mg14 = case14_to_simscape;          % opsiyonel (mg14_params.mat hazır)
cd SIMULINK; rebuild_microgrid_case14
cd ../matlab; runSimulinkGeneratorRecord14
```
Simülatör ve testler (Octave/MATLAB): `cd matlab; run_all_tests14`.

## Sınırlamalar
- `build_transmission14` / `rebuild_microgrid_case14` / `runSimulinkGeneratorRecord14`
  bu ortamda (MATLAB yok) ÇALIŞTIRILMADI; kullanıcı MATLAB'ında ilk koşuda düzeltme gerekebilir.
- Dinamik model klasik (H, D, x'd); AVR/governor/6. mertebe veriler `mg14.exciter/governor` içinde saklı.
- D=0 kondansatörler (17,19,18) zayıf sönümlü salınır.
- Simülasyon yavaştır (5 baralı modelde 10 s ≈ 29 dk).
