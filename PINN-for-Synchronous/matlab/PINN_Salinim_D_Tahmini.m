%[text] # Salınım Denklemi ile Sönüm Katsayısı $D$ Tahmini
%[text] Bu canlı betik (Live Script), bir **senkron elektrik makinesinin** geçici rejim ölçümlerinden sönüm katsayısı $D$'yi, fizik bilgili bir sinir ağı (PINN) ile nasıl tahmin ettiğimizi adım adım gösterir.
%[text] Hedef kitle: lisans elektrik-elektronik mühendisliği. İleri makine öğrenmesi bilgisi gerekmez. Amaç, güç sistemlerindeki klasik **salınım denklemini** (swing equation) bir ters probleme çevirmek ve ağın bu denklemi neden "öğretmen" gibi kullandığını anlamaktır.
%[text] **Öğrenme çıktıları**
%[text] - Senkron makinede atalet sabiti $H$ ve sönüm $D$ ne işe yarar, birimini ve anlamını söyleyebilmek
%[text] - Salınım denklemini per-unit (göreli birim) halde yazabilmek
%[text] - İleri problem (simülasyon) ile ters problem (parametre tahmini) farkını ayırt edebilmek
%[text] - PINN kaybının "veriye uy" ve "fiziğe uy" diye iki parçadan oluştuğunu açıklayabilmek
%[text] - Bir RTDS kaydını yükleyip $D$ tahminini çalıştırabilmek \
%[text:tableOfContents]{"heading": "İçindekiler"}
%%
%[text] ## 1. Fiziksel sahne: senkron makine neden salınır?
%[text] Şebekeye bağlı bir senkron generatör, mekanik güç (türbin veya motor torku) ile elektrik güç (şebekeye verilen tork) arasında bir **denge** arar. Kararlı çalışmada rotor, senkron hızın çok yakınında döner. Bir arıza, yük değişimi veya senkronizasyon olayı bu dengeyi bozar.
%[text] Rotorun mekanik davranışı, lisans makine dersindeki **döner kütle** denklemine benzer: net tork, açısal ivmeyi belirler. Güç sistemlerinde bu denklem çoğunlukla **salınım denklemi** adıyla, göreli birimde yazılır.
%[text] Kullanacağımız büyüklükler:
%[text] - $t$: zaman (s)
%[text] - $\omega$: rotor açısal hızı (per-unit; senkron hızda $\omega = 1$)
%[text] - $T_m$: mekanik tork (per-unit)
%[text] - $T_e$: elektriksel tork (per-unit)
%[text] - $H$: atalet sabiti (s). Rotorun "ne kadar ağır" döndüğünü, yani ivmelenmenin ne kadar yavaş olacağını söyler
%[text] - $D$: sönüm katsayısı (per-unit). Hız senkron değerden sapınca ortaya çıkan yavaşlatıcı / dengeleyici torku temsil eder \
%[text] $D$'yi günlük dille düşünün: kapı yavaş kapanıyorsa menteşede sönüm vardır. Rotor hızı $1$ pu'dan uzaklaşınca $D(\omega-1)$ terimi, sapmayı küçültmeye çalışan bir tork üretir. $D=0$ ise bu yastıklama yoktur; salınım daha uzun sürer.
%%
%[text] ## 2. Salınım denklemi
%[text] Per-unit salınım denkleminin lisans derslerinde en sık görülen hali şudur:
%[text]{"align":"center"} $ 2H \\frac{d\\omega}{dt} = T_m - T_e - D(\\omega - 1) $
%[text] Soldaki $2H\\,d\\omega/dt$ atalet torkudur: hız ne kadar hızlı değişiyorsa atalet o kadar büyük tork ister. Sağdaki $T_m-T_e$ net hızlandırıcı torktur. $D(\\omega-1)$ ise senkron hızdan sapmaya orantılı sönüm torkudur.
%[text] Aynı denklemi **artık** (residual) biçiminde yazalım. Fizik doğruysa artık her anda sıfıra yakındır:
%[text]{"align":"center"} $ r(t) = T_m(t) - T_e(t) - 2H \\dot{\\omega}(t) - D\\bigl(\\omega(t)-1\\bigr) $
%[text] İleri problemde $H$ ve $D$ bilinir, $\\omega(t)$ çözülür (simülasyon). **Ters problemde** $T_m$, $T_e$ ve $\\omega$ ölçülür; bilinmeyen $H$ ve $D$ aranır. Bu betik ikinci işi yapar. Asıl ilgilendiğimiz çıktı $D$'dir; $H$ de birlikte öğrenilir çünkü denklemde ikisi de vardır.
%[text] **Neden $H$ olmadan yalnız $D$ çözmek zordur?** Ölçülen $\\dot{\\omega}$ hem $H$'ye hem $D$'ye bağlıdır. Tek bir salınım kaydında iki bilinmeyen birbirini kısmen taklit edebilir. Bu yüzden kısa eğitimde tahminler başlangıç değerine yakın kalabilir; daha uzun eğitim veya daha zengin geçici rejim gerekir.
%%
%[text] ## 3. PINN nedir? Neden klasik eğri uydurmadan farklıdır?
%[text] Klasik bir sinir ağı yalnızca veriye bakar: "şu anda $T_m$, $\\omega$, $T_e$ böyleydi" der, fiziği bilmez. **Physics-Informed Neural Network (PINN)** hem veriye hem denkleme bakar.
%[text] Ağın girişi zamandır ($t$). Çıkışı üç sinyaldir: $\\hat{T}_m(t)$, $\\hat{\\omega}(t)$, $\\hat{T}_e(t)$. Eğitim sırasında üç kayıp toplanır:
%[text]{"align":"center"} $ L = L\_{\\mathrm{veri}} + L\_{\\mathrm{baslangic}} + L\_{\\mathrm{fizik}} $
%[text] - **Veri kaybı:** ağın çıkışı ölçülen eğrilere benzemelidir.
%[text] - **Başlangıç kaybı:** $t=0$ anındaki değerler ölçüme oturmalıdır.
%[text] - **Fizik kaybı:** artık $r(t)$ sıfıra yaklaşmalıdır. Bu terim $H$ ve $D$'yi de günceller. \
%[text] Böylece ağ "sadece ezberlemez"; tahmin ettiği eğriler salınım denklemiyle de tutarlı olmak zorundadır. $H$ ve $D$ ağın içinde **öğrenilebilir sabitler**dir (DeepXDE'deki `Variable` ile aynı fikir).
%[text] Bu betikteki ağ, Python örneğindeki yapıya yakındır:
%[text] - Giriş dönüşümü: $t$ yerine $\\bigl[0.01t,\\,\\sin(0.01t),\\,\\sin(0.02t),\\dots,\\sin(0.05t)\\bigr]$. Salınımlı işaretleri daha kolay temsil eder.
%[text] - Üç gizli katman, her birinde 50 nöron, **swish** aktivasyonu: $\\mathrm{swish}(z)=z/(1+e^{-z})$.
%[text] - Çıkış dönüşümü: ilk ve son ölçüm noktalarını bir doğru ile birleştirir; ağ yalnızca bu doğrudaki sapmayı öğrenir. Uç noktalar böylece fiziğe zorlanmadan veriye kilitlenir. \
%%
%[text] ## 4. Klasörleri ve bir makine kaydını hazırlayın
%[text] Aşağıdaki kod bu Live Script'in klasörünü yola ekler. Veriler depodaki `data/synchronous-machine` altındadır. Dosya adındaki `H1.5D0.25` ifadesi, RTDS modelinde ayarlanan **gerçek** $H$ ve $D$'dir; bunları doğruluk kontrolü için kullanırız.
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    thisDir = pwd;
end
addpath(thisDir);
repoRoot = fileparts(fileparts(thisDir));
dataDir = fullfile(repoRoot, 'data', 'synchronous-machine');
ornekDosya = fullfile(dataDir, '1208_SyncOn_H1.5D0.25.csv');
%%
%[text] ## 5. Ölçümü yükleyin ve inceleyin
%[text] CSV'ler iki farklı RTDS isimlendirmesi taşıyabilir. `loadSyncCsv` şu öncelikle hız kolonunu seçer: `w_sync`, yoksa `WPU`, yoksa `TESTSPDOUT`. Tork için `Te_sync` veya `TESTTELECT`.
%[text] Zaman ekseninde $T_m$, $\\omega$ ve $T_e$'ye bakın. Senkronizasyon olayında hız $1$ pu çevresinde salınır; elektriksel tork ise ani bir sıçrama yapabilir. PINN tam da bu geçici rejimden $D$'yi okumaya çalışır.
[t, Y, gercekH, gercekD] = loadSyncCsv(ornekDosya, 0, 1);
Tm = Y(:, 1);
w = Y(:, 2);
Te = Y(:, 3);
plot(t, [Tm, w, Te]);
xlabel('Zaman t (s)');
ylabel('Per-unit buyukluk');
legend('T_m mekanik tork', '\\omega rotor hizi', 'T_e elektriksel tork');
title('Ornek RTDS kaydi: SyncOn, H=1.5, D=0.25');
fprintf('Nokta sayisi = %d\n', numel(t));
fprintf('Dosyadan okunan gercek H = %.3f, gercek D = %.3f\n', gercekH, gercekD);
%%
%[text] ## 6. Artığı sayılarla görün
%[text] Eğitimden önce, **gerçek** $H$ ve $D$ ile ölçülen $\\omega$'nın sayısal türevi artığı ne kadar küçültür, onu görelim. Türev gürültülü olduğu için $r(t)$ tam sıfır olmaz; yine de mertebe hissi verir.
%[text]{"align":"center"} $ r\_{\\mathrm{olcum}}(t) = T_m - T_e - 2H\_{\\mathrm{gercek}} \\dot{\\omega}\_{\\mathrm{olcum}} - D\_{\\mathrm{gercek}}(\\omega-1) $
dwdtOlcum = gradient(w, t);
rGercek = Tm - Te - 2 * gercekH * dwdtOlcum - gercekD * (w - 1);
rYanlisD = Tm - Te - 2 * gercekH * dwdtOlcum - 1.0 * (w - 1);
plot(t, [rGercek, rYanlisD]);
xlabel('Zaman t (s)');
ylabel('Artik r(t)');
legend('Gercek D = 0.25 ile artik', 'Yanlis D = 1 ile artik');
title('Dogru D, artigi kucultur (gurultu olsa bile)');
fprintf('Ortalama |artik| (gercek D) = %.4f\n', mean(abs(rGercek)));
fprintf('Ortalama |artik| (D=1)      = %.4f\n', mean(abs(rYanlisD)));
%%
%[text] ## 7. PINN'i eğitin: $H$ ve $D$ öğrenilsin
%[text] Ağ $T_m$, $\\omega$, $T_e$'yi zamana uydururken fizik kaybı $H$ ve $D$'yi de Adam ile günceller. Başlangıç tahmini her ikisi için $1$'dir.
%[text] **İterasyon** kaydırıcısı kısa bir deneme içindir (canlı derste 150–400 yeter). Daha iyi $D$ için 2000–4000 deneyin. Bu Cloud / Octave doğrulamasında kısa bütçe, tahminin *çalıştığını* gösterir; lisans projesinde iterasyonu artırmak sizin işinizdir.
iterasyonSayisi = 200; %[control:slider:iterD]{"position":[20,22]}
ogrenmeOrani = 1e-3;
adim = 2;
tEgitim = t(1:adim:end);
Yegitim = Y(1:adim:end, :);
egitim = struct('iterations', iterasyonSayisi, 'lr', ogrenmeOrani, 'period', max(50, round(iterasyonSayisi / 4)));
[tahminH, tahminD, hist] = trainSwingPINN(tEgitim, Yegitim, egitim);
if abs(gercekD) < 1e-12
    bagilD = abs(tahminD - gercekD);
else
    bagilD = abs(tahminD - gercekD) / abs(gercekD);
end
fprintf('Gercek H = %.3f, tahmin H = %.3f\n', gercekH, tahminH);
fprintf('Gercek D = %.3f, tahmin D = %.3f, bagil/mutlak hata = %.3f\n', gercekD, tahminD, bagilD);
%%
%[text] ## 8. Öğrenme eğrisini okuyun
%[text] Aşağıdaki grafik, eğitim boyunca $H$ ve $D$'nin nasıl yürüdüğünü gösterir. İyi bir çalışmada eğriler gerçek değerlere doğru *yavaşça* kaymalıdır. Kısa iterasyonda çoğu zaman $1$ civarında kalırlar; bu bir yazılım hatası değil, ters problemin zorluğudur.
plot(hist(:, 1), hist(:, 2:3));
hold on
plot(hist([1, end], 1), [gercekH, gercekH], '--');
plot(hist([1, end], 1), [gercekD, gercekD], ':');
hold off
xlabel('Iterasyon');
ylabel('Parametre');
legend('Tahmin H', 'Tahmin D', 'Gercek H', 'Gercek D');
title('H ve D ogrenme egisi');
%%
%[text] ## 9. İsterseniz başka bir makineyi de deneyin
%[text] Aynı fizik, farklı $H$–$D$ çiftleri ve farklı olaylar (yük değişimi `LdChange` veya senkronizasyon `SyncOn`) için geçerlidir. Aşağıda sönümsüz bir vaka ($D=0$) çalışır. $D=0$ iken bağıl hata tanımsızdır; o zaman **mutlak hatayı** kullanırız: $|\\hat{D}-0|$.
ikinciDosya = fullfile(dataDir, '1208_LdChange_H1.0D0.0.csv');
[t2, Y2, gercekH2, gercekD2] = loadSyncCsv(ikinciDosya, 0, 1);
[tahminH2, tahminD2] = trainSwingPINN(t2(1:adim:end), Y2(1:adim:end, :), egitim);
fprintf('LdChange H1.0 D0.0 -> tahmin H = %.3f, tahmin D = %.3f (gercek D = %.3f)\n', tahminH2, tahminD2, gercekD2);
%%
%[text] ## 10. Tüm etiketli makineler (isteğe bağlı)
%[text] On bir `1208_*.csv` dosyasını sırayla eğitmek için yardımcı fonksiyon:
%[text] ```matlab
%[text] runEstimateMachineD('iterations', 400)
%[text] ```
%[text] Sonuç tablosu `results/D_estimates_matlab.csv` dosyasına yazılır. Sütunlar: dosya adı, gerçek $H$/$D$, tahmin $H$/$D$, $D$ hatası.
%[text] **Sonuçları nasıl yorumlamalısınız?**
%[text] - Kısa eğitimde $\\hat{D}\\approx 1$ görmek normaldir; Adam henüz fizik gradyanını biriktirmemiştir.
%[text] - `LdChange` kayıtlarında tork adımı nettir; fizik kaybı genelde daha çabuk düşer.
%[text] - `SyncOn` kayıtlarında $T_e$ çok büyük sıçrayabilir. Veri kaybı baskın kalırsa $D$ yavaş öğrenilir.
%[text] - GTX 950M veya CPU yeterlidir; bu MATLAB kodu ek toolbox istemez. \
%%
%[text] ## 11. Sık sorulan sorular
%[text] **$D$'nin birimi nedir?** Bu formülasyonda per-unit tork / per-unit hız sapmasıdır. SI'da N·m·s/rad gibi görürsünüz; burada tüm torklar zaten taban güce bölünmüştür.
%[text] **Neden $2H$ var?** Klasik derivasyonda rotor kinetik enerjisi $H$ saniye cinsinden tanımlanır; açısal hız denkleminde çarpan $2H$ çıkar. Makine kitabınızdaki "swing equation" bölümüne bakın.
%[text] **PINN, ODE45'ten daha mı doğru?** Hayır, amaç doğruluk yarışı değil. ODE45 ileri problemdir ve $H$, $D$ biliniyorsa daha güvenilirdir. PINN'in işi *bilinmeyen* $D$'yi az veriden tahmin etmektir.
%[text] **Neden Deep Learning Toolbox yok?** Ağ küçük olduğu için ağırlıklar ve Adam adımı düz MATLAB ile yazıldı. Böylece Octave'da da, öğrenci lisansında da açılır.
%%
%[text] ## 12. Mini ödev (ders sonrası)
%[text] 1. `iterasyonSayisi` değerini 1000 yapın. $\\hat{D}$ gerçek $0.25$'e yaklaşıyor mu?
%[text] 2. `1208_SyncOn_H1.0D0.0.csv` dosyasında $D=0$'dır. Tahmin işaret olarak küçük kalıyor mu, yoksa $1$'de mi takılıyor?
%[text] 3. Artık formülünden $D$'yi yalnız ölçümle, en küçük karelerle tahmin edin: $T_m-T_e-2H\\dot{\\omega} \\approx D(\\omega-1)$. PINN ile karşılaştırın.
%[text] 4. $H$'yi dosyadaki gerçek değerde sabitleyip yalnız $D$'yi öğrenmek, çifti birlikte öğrenmekten daha mı kararlı? \

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
%[control:slider:iterD]
%   data: {"defaultValue":200,"label":"Iterasyon sayisi","max":4000,"min":50,"run":"Section","runOn":"ValueChanged","step":50}
%---
