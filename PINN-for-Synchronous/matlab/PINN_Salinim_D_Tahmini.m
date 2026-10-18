%[text] # Salınım Denklemi ile Sönüm Katsayısı $D$ Tahmini
%[text] Bu canlı betik (Live Script), bir **senkron elektrik makinesinin** geçici rejim ölçümlerinden sönüm katsayısı $D$'yi (ve atalet sabiti $H$'yi), fizik bilgili bir sinir ağı (PINN) ile nasıl tahmin ettiğimizi adım adım gösterir.
%[text] Hedef kitle: lisans elektrik-elektronik mühendisliği. İleri makine öğrenmesi bilgisi gerekmez. Amaç, güç sistemlerindeki klasik **salınım denklemini** (swing equation) bir ters probleme çevirmek ve ağın bu denklemi neden "öğretmen" gibi kullandığını anlamaktır.
%[text] **Öğrenme çıktıları**
%[text] - Senkron makinede atalet sabiti $H$ ve sönüm $D$ ne işe yarar, söyleyebilmek
%[text] - Salınım denklemini per-unit (göreli birim) halde yazabilmek ve bir denklemin ölçüme uyup uymadığını **artık (residual)** ile sınayabilmek
%[text] - İleri problem (simülasyon) ile ters problem (parametre tahmini) farkını ayırt edebilmek
%[text] - PINN kaybının "veriye uy" ve "fiziğe uy" diye iki parçadan oluştuğunu açıklayabilmek
%[text] - Bir RTDS kaydını yükleyip $H$ ve $D$ tahminini çalıştırabilmek \
%[text:tableOfContents]{"heading": "İçindekiler"}
%%
%[text] ## 1. Fiziksel sahne: senkron makine neden salınır?
%[text] Şebekeye bağlı bir senkron generatör, mekanik tork (türbin veya motor) ile elektriksel tork (şebekeye verilen tork) arasında bir **denge** arar. Kararlı çalışmada rotor, senkron hızda döner. Bir arıza, yük değişimi veya senkronizasyon olayı bu dengeyi bozar.
%[text] Rotorun mekanik davranışı, lisans makine dersindeki **döner kütle** denklemine benzer: net tork, açısal ivmeyi belirler. Güç sistemlerinde bu denklem **salınım denklemi** adıyla, göreli birimde yazılır.
%[text] Kullanacağımız büyüklükler:
%[text] - $t$: zaman (s)
%[text] - $\omega$: rotor açısal hızı (per-unit; senkron hızda $\omega \approx 1$)
%[text] - $T_m$: mekanik tork (per-unit)
%[text] - $T_e$: elektriksel tork (per-unit; RTDS'in işaret kuralıyla)
%[text] - $H$: atalet sabiti (s). Rotorun "ne kadar ağır" döndüğünü, yani hızın ne kadar yavaş değiştiğini söyler
%[text] - $D$: sönüm katsayısı (per-unit). Hız ile orantılı, yavaşlatıcı tork üretir \
%[text] $D$'yi günlük dille düşünün: kapı yavaş kapanıyorsa menteşede sönüm vardır. $D$ büyüdükçe rotor hızındaki değişimler daha çabuk yatışır; $D=0$ ise bu sürtünme benzeri tork yoktur.
%%
%[text] ## 2. Salınım denklemi ve bu veri setindeki biçimi
%[text] Ders kitaplarında salınım denklemi çoğunlukla şöyle yazılır:
%[text]{"align":"center"} $ 2H \\frac{d\\omega}{dt} = T_m - T_e - D\\,\\omega $
%[text] Soldaki $2H\\,d\\omega/dt$ atalet torkudur: hız ne kadar hızlı değişiyorsa atalet o kadar büyük tork ister. Sağdaki $T_m-T_e$ net hızlandırıcı torktur. $D\\,\\omega$ ise hızla orantılı sönüm torkudur.
%[text] **Dikkat:** bazı kitaplar sönümü $D(\\omega-1)$ yani hız *sapması* üzerinden yazar. Bu iki biçim, $\\omega$ 1 pu'dan çok uzaklaşmadığında bile birbirinden $D$ kadar farklıdır. Hangisinin doğru olduğu, **modelin kendisine** bağlıdır. Bu çalışmada kullandığımız RTDS kayıtlarında sönüm $D\\,\\omega$ ile uygulanmıştır. Bunu aşağıda ölçümle kanıtlayacağız (Bölüm 6).
%[text] Aynı denklemi **artık** (residual) biçiminde yazalım. Fizik doğruysa artık her anda sıfıra yakındır:
%[text]{"align":"center"} $ r(t) = T_m(t) - T_e(t) - 2H \\dot{\\omega}(t) - D\\,\\omega(t) $
%[text] İleri problemde $H$ ve $D$ bilinir, $\\omega(t)$ çözülür (simülasyon). **Ters problemde** $T_m$, $T_e$ ve $\\omega$ ölçülür; bilinmeyen $H$ ve $D$ aranır. Bu betik ikinci işi yapar.
%%
%[text] ## 3. PINN nedir? Neden klasik eğri uydurmadan farklıdır?
%[text] Klasik bir sinir ağı yalnızca veriye bakar: "şu anda $T_m$, $\\omega$, $T_e$ böyleydi" der, fiziği bilmez. **Physics-Informed Neural Network (PINN)** hem veriye hem denkleme bakar.
%[text] Ağın girişi zamandır ($t$). Çıkışı üç sinyaldir: $\\hat{T}_m(t)$, $\\hat{\\omega}(t)$, $\\hat{T}_e(t)$. Eğitim sırasında iki kayıp toplanır:
%[text]{"align":"center"} $ L = L\_{\\mathrm{veri}} + L\_{\\mathrm{fizik}} $
%[text] - **Veri kaybı:** ağın çıkışı ölçülen eğrilere benzemelidir. Her kanal kendi genliğine bölünür; çünkü $\\omega$ 1 civarında çok küçük titreşir, $T_e$ ise $\\pm 1$ salınır.
%[text] - **Fizik kaybı:** artık $r(t)$ sıfıra yaklaşmalıdır. Bu terim $H$ ve $D$'yi de günceller. \
%[text] Böylece ağ "sadece ezberlemez"; tahmin ettiği eğriler salınım denklemiyle de tutarlı olmak zorundadır. $H$ ve $D$ ağın içinde **öğrenilebilir sabitler**dir.
%[text] Bu betikteki eğitim iki aşamalıdır:
%[text] 1. **Adam (ısınma):** yalnızca veri kaybı. Ağ önce eğrilerin şeklini öğrenir.
%[text] 2. **L-BFGS:** veri + fizik kaybı birlikte. Tam veri kümesiyle çalışan, eğrilik bilgisi kullanan bir optimizasyon yöntemidir; $H$ ve $D$ bu aşamada oturur. \
%[text] Ağın yapısı: giriş olarak normalleştirilmiş zaman $\\tau\\in[0,1]$ ve $\\sin(k\\pi\\tau)$, $\\cos(k\\pi\\tau)$ ($k=1,\\dots,12$) değerleri; üç gizli katman, her birinde 50 nöron ve **swish** aktivasyonu $\\mathrm{swish}(z)=z/(1+e^{-z})$. Çıkış, ilk ve son ölçüm noktasını birleştiren doğruya, uçlarda sıfır olan bir düzeltme eklenerek elde edilir.
%[text] **Neden önce veriye uydurup sonra fiziği açıyoruz?** Ağ hızlı salınan $T_e$ eğrisini yeterince keskin öğrenmeden fizik kaybı devreye girerse, ağ $T_e$'yi yumuşatır ve $T_m-T_e$ küçülür. Denklemin sol tarafı o kadar küçük olabilmek için $H$'yi küçültür. Yani yanlış başlangıç, yanlış $H$ demektir.
%%
%[text] ## 4. Klasörleri ve bir makine kaydını hazırlayın
%[text] Live Editor bazen `mfilename` ile geçici bir klasör döndürür; o yüzden yol **bu betikten değil**, yanındaki `loadSyncCsv.m` dosyasından bulunur. `setupSwingPinnPaths` hem yardımcı fonksiyonları MATLAB yoluna ekler hem CSV klasörünü arar.
%[text] Desteklenen iki yerleşim:
%[text] - Git deposu: `PINN-for-Synchronous/matlab/*.m` ve `data/synchronous-machine/*.csv`
%[text] - Tek klasör: `Documents\matlab\*.m` ve `Documents\matlab\data\synchronous-machine\*.csv` \
%[text] `loadSyncCsv.m`, `trainSwingPINN.m`, `parseTrueHD.m` ve `setupSwingPinnPaths.m` **Live Script ile aynı klasörde** olmalıdır.
[matlabDir, dataDir] = setupSwingPinnPaths();
ornekDosya = fullfile(dataDir, '1208_SyncOn_H1.5D0.25.csv');
fprintf('Fonksiyon klasoru: %s\n', matlabDir);
fprintf('Veri klasoru: %s\n', dataDir);
fprintf('Ornek dosya: %s\n', ornekDosya);
%%
%[text] ## 5. Ölçümü yükleyin ve inceleyin
%[text] CSV'ler iki farklı RTDS isimlendirmesi taşıyabilir. `loadSyncCsv` hız kolonunu şu öncelikle seçer: `w_sync`, yoksa `WPU`, yoksa `TESTSPDOUT`. Tork için `Te_sync` veya `TESTTELECT`.
%[text] Aşağıda üç sinyal ayrı paneldedir. Neden? $\\omega$, 1 pu çevresinde yalnızca binde birler mertebesinde değişir; $T_m$ ve $T_e$ ile aynı eksende çizilirse düz bir çizgi gibi görünür. Ortadaki panelde $\\omega-1$ çizilmiştir.
[matlabDir, dataDir] = setupSwingPinnPaths();
ornekDosya = fullfile(dataDir, '1208_SyncOn_H1.5D0.25.csv');
[t, Y, gercekH, gercekD] = loadSyncCsv(ornekDosya, 0, 1);
Tm = Y(:, 1);
w = Y(:, 2);
Te = Y(:, 3);
subplot(3, 1, 1);
plot(t, Tm);
ylabel('T_m (pu)');
title('Ornek RTDS kaydi: SyncOn, H=1.5, D=0.25');
subplot(3, 1, 2);
plot(t, w - 1);
ylabel('\omega - 1 (pu)');
subplot(3, 1, 3);
plot(t, Te);
ylabel('T_e (pu)');
xlabel('Zaman t (s)');
fprintf('Nokta sayisi = %d\n', numel(t));
fprintf('Dosyadan okunan gercek H = %.3f, gercek D = %.3f\n', gercekH, gercekD);
%%
%[text] ## 6. Hangi denklem ölçüme uyuyor? Artıkla sınayalım
%[text] Eğitimden önce, **gerçek** $H$ ve $D$ değerleriyle iki aday denklemi ölçüme uygulayalım. Doğru denklemde artık sıfıra çok yakın olmalıdır. Türevi $\\dot{\\omega}$ ölçümden sayısal olarak alıyoruz.
%[text]{"align":"center"} $ r\_1 = T_m - T_e - 2H\\,\\dot{\\omega} - D\\,\\omega \\qquad r\_2 = T_m - T_e - 2H\\,\\dot{\\omega} - D\\,(\\omega-1) $
dwdtOlcum = gradient(w, t);
r1 = Tm - Te - 2 * gercekH * dwdtOlcum - gercekD * w;
r2 = Tm - Te - 2 * gercekH * dwdtOlcum - gercekD * (w - 1);
ic = 6:numel(t) - 5;
plot(t(ic), [r1(ic), r2(ic)]);
xlabel('Zaman t (s)');
ylabel('Artik r(t)');
legend('r_1: D * \omega (dogru)', 'r_2: D * (\omega - 1)');
title('Hangi denklem olcume uyuyor?');
fprintf('Ortalama |r1| (D*w)     = %.5f\n', mean(abs(r1(ic))));
fprintf('Ortalama |r2| (D*(w-1)) = %.5f   (gercek D = %.2f)\n', mean(abs(r2(ic))), gercekD);
%[text] **Yorum:** $r_1$ sıfıra çok yakındır; $r_2$ ise ortalama olarak tam $D$ kadar sapar. Çünkü $\\omega\\approx 1$ iken $D\\,\\omega - D(\\omega-1) = D$ sabit bir fark bırakır. Bu veriyi $r_2$ ile eğitirseniz, ağ bu sabit farkı kapatmak için $H\\to 0$ ve $D$'yi aşırı büyütmeye çalışır. Yanlış denklem, yanlış parametre demektir.
%%
%[text] ## 7. Ağa danışmadan: doğrudan en küçük kareler
%[text] Sinyaller gürültüsüz olduğundan, $T_m-T_e = 2H\\dot{\\omega} + D\\omega$ ilişkisi $H$ ve $D$ için **doğrusal** bir denklemdir. İki bilinmeyenli bu denklemi en küçük kareler ile çözebiliriz. Bu, PINN'in ulaşması gereken **referans cevaptır**.
A = [2 * dwdtOlcum(ic), w(ic)];
b = Tm(ic) - Te(ic);
katsayilar = A \ b;
fprintf('En kucuk kareler: H = %.4f, D = %.4f  (gercek: H = %.2f, D = %.2f)\n', katsayilar(1), katsayilar(2), gercekH, gercekD);
%[text] **Peki PINN'e neden ihtiyaç var?** Gerçek ölçümlerde $\\dot{\\omega}$'yi sayısal türevle almak gürültüyü büyütür ve bazı sinyaller hiç ölçülmeyebilir. PINN, gürültülü ve eksik veride de çalışan bir ağ kurar; türevi ağdan analitik olarak alır. Bu betikte temiz veri kullandığımız için, PINN'in sonucunu yukarıdaki referansla karşılaştırabiliyoruz.
%%
%[text] ## 8. PINN'i eğitin: $H$ ve $D$ öğrenilsin
%[text] **Adam adımı** ağın eğrileri öğrenmesi içindir (ısınma). **L-BFGS adımı** fizik kaybını da açar ve $H$, $D$ bu aşamada yerine oturur. Eğitim bir RTDS vakası için birkaç on saniye sürer.
%[text] Çok küçük L-BFGS sayısıyla (örneğin 300) $H$ ve $D$ henüz başlangıç değerleri 1'e yakın kalabilir; bu bir hata değil, optimizasyonun bitmemesidir. Kaydırıcıyı artırıp yeniden çalıştırın.
[matlabDir, dataDir] = setupSwingPinnPaths();
iterasyonSayisi = 1000; %[control:slider:iterD]{"position":[19,23]}
lbfgsAdimi = 3000; %[control:slider:lbfgsC]{"position":[14,18]}
adim = 2;
rng(3);
tEgitim = t(1:adim:end);
Yegitim = Y(1:adim:end, :);
egitim = struct('iterations', iterasyonSayisi, 'lbfgsIterations', lbfgsAdimi, 'period', 500);
[tahminH, tahminD, hist] = trainSwingPINN(tEgitim, Yegitim, egitim);
if abs(gercekD) < 1e-12
    bagilD = abs(tahminD - gercekD);
else
    bagilD = abs(tahminD - gercekD) / abs(gercekD);
end
fprintf('Gercek H = %.3f, tahmin H = %.3f\n', gercekH, tahminH);
fprintf('Gercek D = %.3f, tahmin D = %.3f, bagil/mutlak hata = %.3f\n', gercekD, tahminD, bagilD);
%%
%[text] ## 9. Öğrenme eğrisini okuyun
%[text] Üstteki grafik $H$ ve $D$'nin eğitim boyunca yürüyüşünü, alttaki kayıp eğrisini gösterir. İlk bölümde (Adam) $H$ ve $D$ **sabit** durur; çünkü bu aşamada yalnızca veri kaybı vardır ve $H$, $D$ yalnızca fizik kaybında bulunur. Fizik kaybı açıldığında (L-BFGS) ikisi gerçek değerlere doğru kayar.
subplot(2, 1, 1);
plot(hist(:, 1), hist(:, 2:3));
hold on
plot(hist([1, end], 1), [gercekH, gercekH], '--');
plot(hist([1, end], 1), [gercekD, gercekD], ':');
hold off
ylabel('Parametre');
legend('Tahmin H', 'Tahmin D', 'Gercek H', 'Gercek D');
title('H ve D ogrenme egrisi');
subplot(2, 1, 2);
semilogy(hist(:, 1), hist(:, 4));
xlabel('Iterasyon (Adam + L-BFGS)');
ylabel('Toplam kayip L');
%%
%[text] ## 10. İsterseniz başka bir makineyi de deneyin
%[text] Aynı fizik, farklı $H$–$D$ çiftleri ve farklı olaylar (yük değişimi `LdChange` veya senkronizasyon `SyncOn`) için geçerlidir. Aşağıda sönümsüz bir vaka ($D=0$) çalışır. $D=0$ iken bağıl hata tanımsızdır; o zaman **mutlak hatayı** kullanırız: $|\\hat{D}-0|$.
[matlabDir, dataDir] = setupSwingPinnPaths();
ikinciDosya = fullfile(dataDir, '1208_LdChange_H1.0D0.0.csv');
[t2, Y2, gercekH2, gercekD2] = loadSyncCsv(ikinciDosya, 0, 1);
rng(3);
[tahminH2, tahminD2] = trainSwingPINN(t2(1:adim:end), Y2(1:adim:end, :), egitim);
fprintf('LdChange H1.0 D0.0 -> tahmin H = %.3f, tahmin D = %.3f (gercek D = %.3f)\n', tahminH2, tahminD2, gercekD2);
%[text] **Not:** $D$ doğru bulunur ama $H$ gerçek değerden (1,0) uzak çıkar. `LdChange` kayıtlarında $\\omega$ yalnızca $10^{-5}$ mertebesinde oynadığı için atalet terimi $2H\\dot{\\omega}$ neredeyse sıfırdır; $H$ veride neredeyse iz bırakmaz. Bu bir kod hatası değil, o kayıttan $H$'nin **tanımlanamaması**dır. $H$ için `SyncOn` gibi hızın gerçekten değiştiği kayıtlar gerekir.
%%
%[text] ## 11. Tüm etiketli makineler (isteğe bağlı)
%[text] On bir `1208_*.csv` dosyasını sırayla eğitmek için yardımcı fonksiyon:
%[text] ```matlab
%[text] runEstimateMachineD('iterations', 1000, 'lbfgs', 6000)
%[text] ```
%[text] Sonuç tablosu `results/D_estimates_matlab.csv` dosyasına yazılır. Sütunlar: dosya adı, gerçek $H$/$D$, tahmin $H$/$D$, $D$ hatası. Süre: makine başına yaklaşık yarım dakika.
%[text] **Sonuçları nasıl yorumlamalısınız?**
%[text] - $D$ çoğu vakada gerçek değere yakın bulunur. $D=0$ vakalarında mutlak hata küçüktür.
%[text] - `LdChange` kayıtlarında $\\omega$ yalnızca $10^{-5}$ mertebesinde oynar. $D$ ortalama torktan kolayca bulunur, ama $H$ ataletin etkisi görünmediği için **zayıf tanımlanabilir**. Bu fiziksel bir sınırdır, kod hatası değildir.
%[text] - `SyncOn_H1.8D0.2` kaydında hız 0,86 pu'ya kadar düşer ve $T_e$ çok büyür. Bu zor vakada L-BFGS'nin daha çok adım atması gerekir; sonuç ilk denemede daha az doğrudur.
%[text] - Bu MATLAB kodu ek toolbox istemez; CPU yeterlidir. \
%%
%[text] ## 12. Sık sorulan sorular
%[text] **$D$'nin birimi nedir?** Bu formülasyonda per-unit tork / per-unit hızdır. Tüm torklar zaten taban güce bölünmüştür.
%[text] **Neden $2H$ var?** Klasik derivasyonda rotor kinetik enerjisi $H$ saniye cinsinden tanımlanır; açısal hız denkleminde çarpan $2H$ çıkar. Makine kitabınızdaki "swing equation" bölümüne bakın.
%[text] **Önceki sürümde $H\\approx 0$ ve $D\\approx 10$ çıkıyordu. Neden?** Betik o zaman sönümü $D(\\omega-1)$ olarak yazıyordu. Bu RTDS verisi $D\\,\\omega$ ile üretilmiştir (Bölüm 6). Denklem veriye uymayınca fizik kaybı sıfırlanamaz; ağ $H\\to 0$ ve $D$'yi büyüterek artığı kapatmaya çalışır. Ayrıca geri yayılımda $d\\omega/dt$ terimi sabit sayılıyordu; bu da düzeltildi.
%[text] **Neden Deep Learning Toolbox yok?** Ağ küçük olduğu için ağırlıklar, Adam ve L-BFGS adımları düz MATLAB ile yazıldı. Böylece Octave'da da, öğrenci lisansında da açılır.
%%
%[text] ## 13. Mini ödev (ders sonrası)
%[text] 1. Bölüm 6'yı `1208_LdChange_H1.5D0.25.csv` dosyasıyla tekrarlayın. $r_1$ ve $r_2$ yine aynı farkı veriyor mu?
%[text] 2. L-BFGS adımını 300, 1000 ve 3000 yapın. $\\hat{H}$ ve $\\hat{D}$ hangi adımda gerçek değere oturuyor?
%[text] 3. `1208_SyncOn_H1.0D0.0.csv` ile deneyin ($D=0$). Hata neden $D=0.25$ vakasından büyük görünüyor?
%[text] 4. Bölüm 7'deki en küçük kareler yöntemine $\\dot{\\omega}$ için %1 rastgele gürültü ekleyin. $\\hat{H}$ ne kadar bozuluyor? Bu, PINN'in neden anlamlı olduğunu gösterir. \

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
%[control:slider:iterD]
%   data: {"defaultValue":1000,"label":"Adam adimi","max":5000,"min":200,"run":"Section","runOn":"ValueChanged","step":200}
%---
%[control:slider:lbfgsC]
%   data: {"defaultValue":3000,"label":"L-BFGS adimi","max":12000,"min":300,"run":"Section","runOn":"ValueChanged","step":300}
%---
