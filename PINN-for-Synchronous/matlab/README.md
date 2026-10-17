# MATLAB swing-equation PINN

Toolbox gerekmez. MATLAB R2016b+ veya GNU Octave 8 ile çalışır.

## Live Script (önerilen)

MATLAB'de `PINN_Salinim_D_Tahmini.m` dosyasını Live Editor ile açın (R2025a+ düz metin Live Script; daha eski sürümde dosyaya sağ tıklayıp **Open as Live Script**). Tüm başlıklar, formüller ve açıklamalar Türkçedir.

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
