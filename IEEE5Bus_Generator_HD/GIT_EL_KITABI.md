# Git el kitabı (MATLAB Command Window)

Current Folder **her zaman depo kökü** olsun (içinde `.git` var):

`C:\Users\aliaj\Documents\PINN-for-Microgrid-and-Power-Distribution-Systems`

`matlab` veya `outputs` klasöründeyken `git` “not a repository” der. Çıkın:

```matlab
cd('C:\Users\aliaj\Documents\PINN-for-Microgrid-and-Power-Distribution-Systems')
```

Dal: `cursor/swing-pinn-d-estimate-ef5e`  
(`main`’de bu betikler yok / eksik olabilir.)

---

## 1) Bilgisayara al (PULL) — senden GitHub’a bak

“Cursor / PR’da yeni kod var, lokalime gelsin.”

```matlab
cd('C:\Users\aliaj\Documents\PINN-for-Microgrid-and-Power-Distribution-Systems')
!git fetch origin
!git checkout cursor/swing-pinn-d-estimate-ef5e
!git pull origin cursor/swing-pinn-d-estimate-ef5e
!git status
```

`git status` temiz (`working tree clean`) ise güncelsiniz.

---

## 2) GitHub’a gönder (PUSH) — senden GitHub’a yaz

“CSV / kod ürettim, repoya gitsin.”

```matlab
cd('C:\Users\aliaj\Documents\PINN-for-Microgrid-and-Power-Distribution-Systems')
!git status
!git add IEEE5Bus_Generator_HD/outputs/data/Simulink_SM_Bus*.csv IEEE5Bus_Generator_HD/outputs/simulink_HD_estimates.csv
!git commit -m "Add Simulink SM_Bus recordings"
!git push origin cursor/swing-pinn-d-estimate-ef5e
```

Tüm değişen dosyalar (dikkatli kullanın):

```matlab
!git add -A
!git commit -m "Aciklayici mesaj"
!git push origin cursor/swing-pinn-d-estimate-ef5e
```

Simulink betiği commit’i sizin yerinize dener; push için:

```matlab
cd('IEEE5Bus_Generator_HD\matlab')
runSimulinkGeneratorRecord(struct('gitPush', true))
```

---

## Tek bakışta

| Ne istiyorsunuz | Komut özeti |
|---|---|
| GitHub’daki hali al | `fetch` → `checkout` (dal) → `pull` |
| Lokal kaydı GitHub’a yaz | `add` → `commit` → `push` |
| Neredeyim? | `!git branch --show-current` |
| Ne değişti? | `!git status` |

Sıra: **önce pull, işini yap, sonra add/commit/push.** Tersine çevirmeyin (eski hali üzerine yazarsınız).

---

## Takılırsa (kısa)

| Mesaj | Ne yapın |
|---|---|
| `not a git repository` | `cd` depo köküne |
| `Please commit your changes or stash them` | Önce `status`. CSV’leri `add`+`commit` veya `!git stash` |
| `nothing to commit` | Yeni dosya yok; `add` etmeyi unutmuş olabilirsiniz |
| `rejected` / `non-fast-forward` | Önce `!git pull origin cursor/swing-pinn-d-estimate-ef5e`, sonra `push` |
| `git` tanınmıyor | Git for Windows kurulu + PATH; MATLAB’i yeniden açın |
| Push kimlik soruyor | GitHub oturumu (tarayıcı / Git Credential Manager) — bir kez |

`pull` = oku. `push` = yaz. İkisini karıştırmayın.
