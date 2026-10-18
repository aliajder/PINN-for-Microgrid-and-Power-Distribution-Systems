# MATLAB swing-equation PINN

Toolbox gerekmez. MATLAB R2016b+ veya GNU Octave 8 ile çalışır.

## Live Script (önerilen)

MATLAB'de `PINN_Salinim_D_Tahmini.m` dosyasını Live Editor ile açın (R2025a+ düz metin Live Script; daha eski sürümde dosyaya sağ tıklayıp **Open as Live Script**). Tüm başlıklar, formüller ve açıklamalar Türkçedir.

Yardımcı fonksiyonlar (`loadSyncCsv.m`, `trainSwingPINN.m`, `setupSwingPinnPaths.m`) Live Script ile **aynı klasörde** durmalıdır. Veri için iki düzen geçerlidir:

```
Documents\matlab\PINN_Salinim_D_Tahmini.m
Documents\matlab\loadSyncCsv.m
Documents\matlab\data\synchronous-machine\1208_SyncOn_H1.5D0.25.csv
```

veya git deposundaki orijinal `data/synchronous-machine` konumu. `setupSwingPinnPaths` her iki yolu da arar; Live Editor geçici klasör kullansa bile `loadSyncCsv.m`'nin durduğu yeri MATLAB path'ine ekler.

```matlab
cd PINN-for-Synchronous/matlab
edit PINN_Salinim_D_Tahmini.m
```

## Toplu tahmin

```matlab
cd PINN-for-Synchronous/matlab
runEstimateMachineD
runEstimateMachineD('iterations', 4000)
runEstimateMachineD('cases', {'SyncOn_H1.0D0.0', 'SyncOn_H1.5D0.25'})
```

Çıktı: `results/D_estimates_matlab.csv`.
