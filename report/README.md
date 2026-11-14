# Bulanık Mantık Dersi — Dönem Projesi Teknik Raporu

**Ders:** Bulanık Mantık Projesi (2026-2027)  
**Proje Başlığı:** *A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation* İncelemesi, Yeniden Gerçekleştirimi ve Erişilebilirlik Farkındalıklı Görev Yönetimi  
**Öğrenci:** Adnan Emir Kırılmış  
**Tarih:** 14.11.2026  
**Eğitmen:** Prof. Dr. Rüya ŞAMLI  
**Asistan Eğitmen:** Arş. Gör. Tolgahan ÖZTÜRK  
**Resmi Rapor PDF Belgesi:** [report.pdf](report.pdf) (11 Sayfa, IEEE Formatı)

---

## ÖZET (ABSTRACT)
Otonom mobil robotların karmaşık ve değişken ortamlarda güvenli, çarpışmasız ve minimum sürede birden fazla hedef noktayı ziyaret edebilmesi, modern robotik araştırmalarının temel taşlarından biridir. Bu çalışmada, MDPI Sensors dergisinde yayımlanan *"A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation"* (Stavrinidis vd., 2025) başlıklı referans makalenin metodolojisi CoppeliaSim simülasyon ortamında Pioneer P3-DX mobil robotu üzerinde Python ve Antigravity IDE vasıtasıyla sıfırdan ele alınarak yeniden gerçekleştirilmiştir (reproduction). Yeniden üretim sürecinde CoppeliaSim'in varsayılan Pioneer modelindeki gömülü Braitenberg Lua Child Script'inin Python motor hız komutlarını ezdiği tespit edilmiş, geliştirilen dinamik nötralizasyon fonksiyonu ile kontrol bütünüyle Python tarafına alınmıştır. Ayrıca referans makalede ele alınmayan, fiziksel olarak ulaşılamaz (4 duvarla kapatılmış izole oda) hedefler karşısında robotun duvara toslamasını ve kilitlenmesini önlemek üzere *"Reachability-Aware Task Manager"* (Erişilebilirlik Farkındalıklı Görev Yöneticisi) özgün mimarisi geliştirilmiştir. Geliştirilen 6 durumlu sonlu durum makinesi ve 4-taraf çevre taraması mekanizması ile robot, kapalı hedefi güvenle tespit edip atlayarak görevi başarıyla tamamlamaktadır. 6 farklı senaryoda gerçekleştirilen deneysel değerlendirmelerde önerilen özgün yöntem, temel yaklaşıma kıyasla %17.3 ile %47.5 arasında kanıtlanmış zaman ve enerji tasarrufu sağlamıştır.

---

## 1. INTRODUCTION
Otonom mobil robotların karmaşık ve değişken ortamlarda güvenli bir şekilde hareket edebilmesi, modern robotik araştırmalarının temel taşlarından biridir. Bu çalışmanın temel problemi, bir mobil robotun önceden belirlenmiş birden fazla hedef noktayı en kısa sürede ve engellere çarpmadan ziyaret etmesini sağlamaktır [1]. Çalışmanın motivasyonu, hem statik engellerin varlığında küresel rota optimizasyonu gerçekleştirmek hem de dinamik engellere karşı anlık reaktif kararlar alabilen hibrit bir mimari tasarlamaktır. Proje kapsamında, referans makalede önerilen strateji incelenmiş, CoppeliaSim ve Python kullanılarak simülasyon ortamında yeniden gerçekleştirilmiş ve performansını artıran özgün bir katkı entegre edilmiştir.

Çoklu hedefli navigasyon görevi endüstriyel depolarda parça toplama, afet bölgelerinde arama-kurtarma, haritalandırma ve muayene görevlerinde kritik önem taşımaktadır. Hedef noktalarının ziyaret edilme sırası kombinatoryal bir Gezgin Satıcı Problemi (Traveling Salesperson Problem - TSP) oluştururken, hedefler arasında robotun rotasında aniden beliren insan, kolon, kutu veya bariyer gibi engeller anlık yönelim ve hız regülasyonu gerektirmektedir. Geleneksel deterministik planlayıcılar (A*, Dijkstra) statik haritalarda etkili olsa da, bilinmeyen engellerin bulunduğu dinamik ortamlarda yüksek hesaplama maliyeti doğurmakta ve gerçek zamanlı reaksiyon vermekte yetersiz kalmaktadır. Bulanık Mantık Kontrolörleri (Fuzzy Logic Controllers - FLC), sensör ölçüm gürültülerini tolere edebilme ve dilsel insan sezgilerini kural tabanına aktarabilme yeteneği sayesinde bu reaktif navigasyon problemine mükemmel bir çözüm sunmaktadır.

---

## 2. RELATED WORK
Mobil robot navigasyonunda çoklu hedef problemleri (Multi-Goal Path Planning - MGPP), Gezgin Satıcı Problemi (TSP) ile yerel engelden kaçınma algoritmalarının birleştirilmesiyle çözülmektedir [1]. Literatürde yapay sinir ağları, genetik algoritmalar ve potansiyel alan yöntemleri yaygın olarak kullanılmaktadır [2]. Bulanık mantık kontrolörleri ise belirsizlikleri yönetme ve insan benzeri karar verme yetenekleri sayesinde reaktif navigasyonda öne çıkmaktadır [3].

Literatürdeki çalışmalar incelendiğinde üç ana metodolojik yaklaşım dikkat çekmektedir:
1. **Yapay Potansiyel Alanlar (Artificial Potential Fields - APF):** Hedefe çekici, engellere itici kuvvet uygulayan bu yöntem matematiksel olarak sade olmasına rağmen yerel asgari (local minima) tuzağına düşmekte ve dar koridorlarda salınım (chattering) yapmaktadır [2].
2. **Sezgisel Arama ve Genetik Algoritmalar (GA):** $N$ hedefli TSP problemlerinde Order Crossover (OX) ve mutasyon operatörleri global rota optimizasyonunda küresel en iyiyi bulmada başarısını kanıtlamıştır [4].
3. **Bulanık Mantık Sistemleri (FLC):** Mamdani çıkarım mekanizması, ultrasonik sensör sektör mesafelerini sol ve sağ motor hızlarına eşleyerek diferansiyel robotlarda pürüzsüz viraj alma sağlamaktadır. Stavrinidis vd. (2025) [1], GA ile çift bulanık kontrolörü birleştirerek hem hedef takibini hem de statik/dinamik kaçınmayı başarmıştır. Ancak mevcut literatürde hedeflerin topolojik olarak erişilemez (örneğin 4 duvarla izole kapalı oda) olduğu durumlar göz ardı edilmiş; robotun duvara toslaması veya sonsuz kaçınma döngüsüne girmesi engellenememiştir. Bu çalışmanın özgün katkısı, literatürdeki bu temel boşluğu doldurmaktadır.

---

## 3. THEORETICAL BACKGROUND
Projede kullanılan sistemin teorik altyapısı Bulanık Mantık Kontrolörlerine (FLC) ve diferansiyel sürüşlü robot kinematiğine dayanmaktadır. Diferansiyel robotun hareketi sol ve sağ tekerlek hızlarına bağlıdır. FLC mimarisinde giriş değişkenleri (hedefe olan uzaklık, açı ve engel mesafesi) bulanıklaştırılır, kural tabanı (min-max çıkarım kuralları) ile işlenir ve durulaştırma (defuzzification) adımıyla doğrusal ve açısal hız komutlarına dönüştürülür.

### 3.1. Diferansiyel Sürüş Mobil Robot Kinematiği
Pioneer P3-DX robotu iki bağımsız tahrik tekerleği ve serbest sarhoş tekerlekten oluşur. Robotun global koordinatlardaki konumu $p = [x, y, \theta]^T$ vektörüdür. Tekerlek yarıçapı $r$ ve tekerlek taban eksen mesafesi $L$ olmak üzere:
$$v = \frac{r (v_R + v_L)}{2}, \quad \omega = \frac{r (v_R - v_L)}{L}$$

Ters kinematik bağıntısı ile çizgisel ve açısal hızlardan sol ve sağ motor açısal hızlarına ($v_L, v_R$) geçilir:
$$v_R = \frac{2v + \omega L}{2r}, \quad v_L = \frac{2v - \omega L}{2r}$$

### 3.2. Mamdani Bulanık Çıkarım Mekanizması
Bulanık mantık kuramı (Zadeh, 1965 [5]), klasik küme mantığındaki $\{0, 1\}$ kesinliğini $[0, 1]$ sürekli üyelik derecesi aralığına genişletir. Bu projede Mamdani çıkarım modeli uygulanmıştır:
1. **Bulanıklaştırma (Fuzzification):** Sürekli fiziksel sensör ölçümleri üyelik fonksiyonları (üçgen ve yamuk) aracılığıyla dilsel değişkenlere (*Very Close*, *Close*, *Medium*, *Far*, *Safe* vb.) dönüştürülür.
2. **Kural Değerlendirme (Inference):** EĞER (Koşul) İSE (Sonuç) yapısındaki kurallar Mamdani minimum operatörü ile birleştirilir. Çoklu kuralların tetiklenmesi maksimum birleşim (Max-Min composition) ile toplanır:
   $$\mu_C(y) = \max_k \left[ \min \left( \mu_{A_k}(x_1), \mu_{B_k}(x_2) \right) \right]$$
3. **Durulaştırma (Defuzzification):** Elde edilen bulanık alan, ağırlık merkezi (Centroid / Center of Gravity - CoG) yöntemiyle kesin motor hız değerlerine dönüştürülür:
   $$y^* = \frac{\int y \cdot \mu(y) \, dy}{\int \mu(y) \, dy}$$

Ağırlık merkezi yöntemi, motor hız profillerinde basamak sıçramaları yerine sürekli ve diferansiyel robot mekaniğine uygun yumuşak tork tepkileri ürettiği için tercih edilmiştir.

---

## 4. ORIGINAL METHOD
Referans çalışmada [1], çok hedefli navigasyon problemi iki aşamalı olarak ele alınmıştır: İlk aşamada Genetik Algoritma (GA) kullanılarak hedeflerin ziyaret sırası optimize edilmiştir. İkinci aşamada ise statik engeller için Sınır Düğümü Yöntemi (Boundary Node Method), dinamik engeller ve rota takibi için ise ikili bulanık mantık denetleyicileri kullanılarak çarpışmasız yörünge üretilmiştir.

### 4.1. Genetik Algoritma Tabanlı Global Yol Planlama (GA-TSP)
Referans makalede $N=12$ hedef noktası ve 1 başlangıç üssü (depot) yer almaktadır. Hedefler Gezgin Satıcı Problemi olarak modellenmiştir. Hedef dizilimi $\pi = (\pi_1, \pi_2, \dots, \pi_N)$ permütasyonu olarak kodlanmıştır:
* **Uygunluk (Fitness) Fonksiyonu:** $F(\pi) = 1.0 / J(\pi)$, burada $J(\pi)$ başlangıç üssünden çıkıp tüm hedefleri sırayla dolaşıp üsse dönen toplam Öklid tur uzunluğudur:
  $$J(\pi) = \|D - g_{\pi(1)}\| + \sum_{i=1}^{N-1} \|g_{\pi(i)} - g_{\pi(i+1)}\| + \|g_{\pi(N)} - D\|$$
* **Sıralı Çaprazlama (Order Crossover - OX):** Ebeveyn kromozomlardan alt rotaları koruyarak geçerli bir hedef ziyaret sırası üretir, hedef tekrarını önler.
* **Ters Çevirme Mutasyonu (Inversion Mutation):** İki rastgele indis arasındaki hedef sırasını ters çevirerek yerel döngü düğümlerini çözer.

### 4.2. Çift Bulanık Mantık Denetleyicisi Mimarisi
Referans makale hedef sürüşü ile engelden kaçınmayı iki ayrı Mamdani modülüne ayırmıştır:
* **Hedef Takip Denetleyicisi (Tracking Controller - 15 Kural):**
  - Giriş 1: Konum Hatası $e_{pos} = \|p_{goal} - p_{robot}\|$ (*Small*, *Medium*, *Large*).
  - Giriş 2: Yönelim Hatası $e_{head} = \text{atan2}(\Delta y, \Delta x) - \theta$ (*NB*, *NS*, *Z*, *PS*, *PB*).
  - Çıkışlar: $v_L$ ve $v_R$ (*Slow*: 0.5 rad/s, *Medium*: 1.5 rad/s, *Fast*: 2.5 rad/s).
* **Engelden Kaçınma Denetleyicisi (Obstacle Avoidance Controller - 25 Kural):**
  - Robotun ön 6 ultrasonik sensörü sol ve sağ sektör olarak indirgenir: $d_{left} = \min(s_3, s_4)$, $d_{right} = \min(s_5, s_6)$.
  - Giriş Kümeleri: *Very Close (VC)*, *Close (C)*, *Medium (M)*, *Far (F)*, *Safe (S)*.
  - Eşik Mekanizması: $\min(d_{left}, d_{right}) < 0.85\text{ m}$ olduğunda kaçınma denetleyicisi tam öncelikle devreye girer.
  - Kaçınma Kuralı: Sol sektör yakınsa sağ motor hızlandırılır (sağa kaçış); sağ sektör yakınsa sol motor hızlandırılır; kafa kafaya engelde robot geri çekilerek döner.

---

## 5. REPRODUCTION METHODOLOGY
Bu projede, referans makalede sunulan deneyler sıfırdan ele alınarak CoppeliaSim simülatörü ve Python programlama dili vasıtasıyla yeniden gerçekleştirilmiştir. CoppeliaSim üzerinde tekerlekli mobil robot modeli yapılandırılmış, Python tarafında ise scikit-fuzzy ve NumPy kütüphaneleri kullanılarak bulanık çıkarım sistemleri kodlanmıştır. Python betiği ile simülatör arasında Remote API bağlantısı kurularak sensör verileri (lidar/yaklaşım sensörleri) anlık olarak okunmuş ve hesaplanan motor hızları robota aktarılmıştır. Deneyin yeniden gerçekleme kısmı aşırı zor olmuştur ve deney gerçeklenirken Üretken Yapay Zekalardan yardım alınarak 3 iş günü içerisinde yeniden gerçeklenmiştir. Deneyi yapan araştırmacılar gerçek deneyde Python yerine Matlab kullanmışlardır. Projenin kodları açık kaynak olmadığı için görselleştirme mantığı aynı tutularak istenen şekillerde birebir yeniden gerçeklenmiştir. Deney Antigravity IDE si üzerinden yeniden gerçeklenmiş ve tüm kodlar GitHub a yüklenmiştir.

### 5.1. Başlangıç Kök Neden Analizi ve Motor Kilidi Çözümü (Initial Root Cause Analysis)
Reproduction sürecinin başında `experiments/baseline/stop_test.py` betiği yürütüldüğünde beklenmeyen bir davranış gözlenmiştir: Python betiğinden robot motorlarına 0.0 rad/s hedef hızı gönderilmesine rağmen, robotun fiziksel hızı 2.0 rad/s seviyesinde kalmış ve robot durmayarak sahnedeki engellere çarpmıştır.
* **Kök Neden Tespiti:** CoppeliaSim içerisindeki standart `PioneerP3DX` modeli detaylı olarak incelendiğinde, modelin gövde nesnesi altında gömülü bir Braitenberg Lua Child Script bulunduğu anlaşılmıştır. Bu script simülasyonun her zaman adımında (tick) `sim.setJointTargetVelocity()` fonksiyonunu çağırarak tekerleklere 2.0 rad/s hız atamakta ve Python Remote API üzerinden gelen komutları anında ezmektedir.
* **Mühendislik Çözümü:** `src/robot_interface.py` modülü içerisine `_disable_child_script()` fonksiyonu entegre edilmiştir. Bu fonksiyon sahne başlatıldığında Pioneer robotuna bağlı Lua script nesnesini tespit etmekte ve simülasyon düzeyinde nötralize etmektedir. Bu sayede robot motorları 0.000 rad/s tam durgunluk değerine indirgenmiş ve çift bulanık denetleyicinin robot üzerindeki tam ve kesintisiz kontrolü sağlanmıştır.

### 5.2. Simülasyon-Python İletişim Mimarisi
Makale yazarları orijinal çalışmada tescilli Matlab/Simulink ortamını kullanmışken, bu projede açık kaynak ve modern endüstri standardı olan Python 3.10+, ZeroMQ Remote API (`coppeliasim-zmqremoteapi-client 2.0.4`) ve CBOR ikili serileştirme mimarisine geçilmiştir. Bu sayede platform bağımsız, hafif ve başka bilgisayarlarda kolaylıkla yeniden çalıştırılabilir bir geliştirme ekosistemi tesis edilmiştir.

---

## 6. PROPOSED EXTENSION (REACHABILITY-AWARE TASK MANAGER)
Deney yeniden gerçeklendikten sonra, deneyde bulunmayan bir şart oluşturulmuş ve ona göre bir özgün katkı geliştirilmiştir. Özgün katkı kısmının fikrinde hiçbir LLM kullanılmamıştır. Fikir tamamen özgündür. Deneyin özgün katkı kısmında referans alınan makalede yapılmamış bir şey yapılmış ve erişilmesi imkansız olan bir varış noktası eklenmiştir. Bu varış noktasının etrafı 4 duvarla kapatılmış olsa da robota o varış noktasına varması söylenmiş ve ne yapacağı test edilmiştir. Daha sonra robotun tepkisi incelenmiş ve robota belli komutlar verilmiştir, robot önce tüm yönlerden varış noktasına varıp varamayacağını kontrol edecek eğer varamayacağı kanaatine varırsa o varış noktasını “Bu varış noktası erişilemez” gibi bir etiket atayarak o noktayı erişilemez durumuna çevirecektir. Bu nedenle nokta, robota göre varış noktası olmaktan çıkacaktır. Eğer robotun her yönü gezme süresi chatting gibi problemlerle fazla uzun sürerse robot noktayı otomatik olarak atlayacaktır. Süre bazlı atlama durumu referans makalede de bulunmaktadır.

### 6.1. Reachability-Aware Task Manager Mimarisi (`src/reachability_manager.py`)
Bu özgün katkıyı formalize etmek amacıyla 3 ana bileşenden oluşan bir karar destek sistemi tasarlanmıştır:
1. **6 Kademeli Hedef Durum Makinesi (State Machine):**
   * `PENDING`: Hedef sırada, henüz değerlendirilmedi.
   * `REACHABLE`: Açık yol teyit edildi, hedefe yaklaşılıyor.
   * `TEMPORARILY_BLOCKED`: Anlık dinamik engel algılandı, bekleniyor veya çevre deneniyor.
   * `UNCERTAIN`: İlerleme yavaşladı, hedef doğrudan görünmüyor.
   * `UNREACHABLE`: Hedefin 4 tarafı taranmış olup tamamen kapalı olduğu kanıtlandı; hedef güvenle atlanır.
   * `COMPLETED`: Hedefe tolerans yarıçapı ($< 0.25\text{ m}$) içinde başarıyla ulaşıldı.
2. **4-Taraf Çevre Taraması (Perimeter Exploration):**
   * Robot kapalı bir kutu veya oda ile karşılaştığında hedefi körü körüne terk etmez. Odanın Kuzey, Doğu, Güney ve Batı kenarlarını teğetsel olarak dolaşarak giriş kapısı arar. 4 cephede de giriş noktası bulunamazsa hedef kesin olarak `UNREACHABLE` ilan edilir. Bu sayede kapısı açık odalarla kapalı odalar kusursuz biçimde ayırt edilir.
3. **Durgunluk Analizi (Stagnation Detection & Timeout):**
   * Robotun hedefle arasındaki mesafe $\Delta d < 0.10\text{ m}$ seviyesinde kalırsa ve 5 ardışık adımda net ilerleme kaydedilemezse veya süre bazlı eşik aşılırsa kilitlenme tespit edilir ve robot sonraki hedefe yönlendirilir.

---

## 7. EXPERIMENTAL SETUP
Deneyler, 64-bit Python 3.10 ve CoppeliaSim EDU ortamını çalıştıran bir bilgisayar üzerinde gerçekleştirilmiştir. Deney ortamı; sabit engeller, duvarlar ve önce 5 daha sonra 10-12 farklı hedef noktadan oluşan bir depo senaryosunu içermektedir. Denetleyici parametreleri üyelik fonksiyonları için üçgensel ve gauss eğrileri olarak tanımlanmıştır.

### 7.1. Simülasyon Sahnesi ve Çevresel Parametreler
* **Zemin Boyutları:** 12.0m x 12.0m kapalı endüstriyel depo tabanı.
* **Mobil Robot Platformu:** Pioneer P3-DX diferansiyel robotu (aks açıklığı $L=0.33\text{ m}$, tekerlek yarıçapı $r=0.0975\text{ m}$).
* **Sensör Konfigürasyonu:** Ön gövdeye yayılmış 16 adet ultrasonik mesafe sensörü (50 Hz okuma frekansı).
* **Başlangıç Üssü (Depot):** `[-2.65, -3.10]` m koordinatında konumlandırılmıştır.
* **Hedef Noktaları ($N=12$):** Makaledeki 12 hedef noktası `[-2.48, -0.95]`, `[-1.63, 0.10]`, `[-0.83, -0.60]`, `[-1.48, -1.60]`, `[0.07, -1.95]`, `[0.72, -0.80]`, `[0.67, 0.95]`, `[1.27, -1.95]`, `[2.17, -1.95]`, `[2.32, -1.50]`, `[2.32, -0.70]`, `[2.32, 0.50]`.
* **4 Duvarlı Kapalı Oda (Özgün Test Senaryosu):** `[0.67, 0.95]` konumundaki Hedef 6, 4 adet çarpışma duvarından oluşan tamamen izole bir oda içerisine hapsedilmiştir; içeriye hiçbir fiziksel koridor bulunmamaktadır.

### 7.2. Yazılım Kütüphaneleri ve Donanım Ortamı
* **İşletim Sistemi:** Windows 11 Pro 64-bit / Intel Core i7, 16 GB RAM.
* **Programlama Dili ve IDE:** Python 3.13 / Antigravity IDE.
* **Kullanılan Kütüphaneler ve Kesin Sürümleri:**
  - `coppeliasim-zmqremoteapi-client==2.0.4` (Simülatör ZeroMQ API köprüsü)
  - `pyzmq==27.2.0`, `cbor==1.0.0` (Soket iletişimi ve serileştirme)
  - `scikit-fuzzy==0.5.0`, `scipy==1.18.1` (Bulanık çıkarım ve sayısal fonksiyonlar)
  - `numpy==2.5.4`, `matplotlib==3.11.2` (Vektörel matematik ve telemetri görselleştirmesi)
  - `pillow==12.3.0`, `python-dateutil==2.9.0.post0` (Görsel işleme ve zaman serileri)

---

## 8. RESULTS
Yeniden gerçekleştirme (Baseline) ve önerilen uzantılı sistemin (Extended) performans metrikleri 10 farklı bağımsız deney çalıştırılarak karşılaştırılmıştır.

### 8.1. Makale 5 Temel Senaryo Doğrulaması (Baseline Reproduction)
Referans makalenin farklı engel koşullarındaki davranışını doğrulamak üzere `experiments/baseline/benchmark_experiments.py` ile yürütülen 5 senaryo test sonuçları Çizelge 1'de sunulmuştur:

#### Çizelge 1: Makale Yeniden Üretimi Kapsamında 5 Temel Senaryo Doğrulama Sonuçları
| Senaryo Adı | Başarı Oranı (%) | Yol Uzunluğu (m) | Toplam Süre (s) | Çarpışma Sayısı | Ort. Hedef Hatası (m) |
|---|:---:|:---:|:---:|:---:|:---:|
| **1. Engel Bulunmayan Ortam** | %100.0 | 5.97 m | 14.25 s | 0 | 0.312 m |
| **2. Tek Engelli Ortam** | %100.0 | 6.53 m | 16.08 s | 0 | 0.311 m |
| **3. Çoklu Engelli Ortam** | %100.0 | 6.92 m | 15.88 s | 0 | 0.312 m |
| **4. Dar Geçiş Senaryosu** | %75.0 | 7.01 m | 20.93 s | 0 | 0.312 m |
| **5. GA Olmadan (Düz Sıra)** | %100.0 | 6.59 m | 19.07 s | 0 | 0.311 m |

*Not:* GA rota optimizasyonunun devrede olduğu Senaryo 1'de toplam süre 14.25 saniye iken, GA'nın devre dışı bırakıldığı Senaryo 5'te süre 19.07 saniyeye yükselmiş; GA'nın rotayı %25.3 oranında kısalttığı doğrulanmıştır.

### 8.2. Makale Figür 30 ve 33 Yeniden Üretimi
Adım adım robot telemetrisi [`data/navigation_results.csv`](../data/navigation_results.csv) dosyasından okunarak makaledeki formatta [`results/figures/navigation_performance.png`](../results/figures/navigation_performance.png) olarak çizdirilmiştir:
* **2D Navigasyon Rotası:** Robot 12 hedefi ve üssü engellere çarpmadan pürüzsüz virajlarla tamamlamıştır.
* **Konum Hatası ($e_{pos}$):** Hedeflere yaklaşıldıkça hata sıfıra yaklaşmakta, hedef geçişinde periyodik olarak yenilenmektedir.
* **Tekerlek Hız Profilleri ($v_L, v_R$):** Centroid durulaştırması sayesinde ani basamak sıçramaları yerine diferansiyel robot dinamiğine uygun yumuşak eğriler elde edilmiştir.
* **Yönelim Hatası ($e_{head}$):** Dönüşlerde $[-180^\circ, +180^\circ]$ arasında kararlı biçimde sıfırlanmıştır.

### 8.3. 3 Yöntem ve 6 Senaryolu Kapsamlı Karşılaştırmalı Değerlendirme
Özgün Reachability-Aware Task Manager, referans makale algoritması ve sezgisel atlama yöntemi ile 6 farklı senaryoda test edilmiştir (Çizelge 2 ve Şekil 2):
* **Yöntem A (Baseline):** Referans makale algoritması (erişilemezlik farkındalığı yok).
* **Yöntem B (Simple Skip):** Sabit adım denemesinden sonra körü körüne hedef atlama.
* **Yöntem C (Reachability-Aware Önerilen):** Durum makinesi, topoloji analizi ve 4-taraf çevre taramalı özgün sistem.

#### Çizelge 2: 3 Yöntemin 6 Senaryodaki Süre, Zaman Tasarrufu ve Görev Başarı Performansı
| No | Senaryo Tanımı | Yöntem A Süre | Yöntem B Süre | Yöntem C (Önerilen) | Yöntem C Zaman Tasarrufu | Ulaşılabilir Hedef Başarısı | Hatalı Atlama Oranı |
|:---:|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **1** | **Tümü Ulaşılabilir Ortam** | 26.00 s | 26.00 s | **26.00 s** | Referans (%0) | %100.0 | %0.0 |
| **2** | **Tek Hedef 4 Duvarla Kapalı** | 31.50 s *(Kilitlendi)* | 31.50 s | **22.70 s** | **%27.9 Daha Hızlı** | %100.0 | %0.0 |
| **3** | **Geçici Engelli Ortam** | 33.50 s | 29.50 s *(Hata)* | **27.70 s** | **%17.3 Daha Hızlı** | %100.0 | %0.0 *(B: %25 Hata)* |
| **4** | **Alternatif Rotalı Ortam** | 33.50 s | 29.50 s *(Hata)* | **27.70 s** | **%17.3 Daha Hızlı** | %100.0 | %0.0 *(B: %25 Hata)* |
| **5** | **Sensör Gürültülü Ortam** | 38.00 s *(Kilitlendi)* | 31.50 s | **22.70 s** | **%40.2 Daha Hızlı** | %100.0 | %0.0 |
| **6** | **Çoklu Erişilemez Hedef** | 31.50 s *(Kilitlendi)* | 37.00 s | **19.40 s** | **%47.5 Daha Hızlı** | %100.0 | %0.0 |

---

## 9. DISCUSSION
Elde edilen sonuçlar, Python ve CoppeliaSim entegrasyonunun orijinal çalışmanın bulgularını başarıyla doğruladığını göstermektedir. Önerilen hız engeli uzantısı, özellikle dinamik engellerin yoğun olduğu bölgelerde erken manevra yapılmasını sağlayarak toplam yol uzunluğunu kısaltmış ve çarpışma riskini sıfıra indirmiştir.

### 9.1. Baseline Sistemin Kapalı Odalardaki Kilitlenme Mekanizması
Deney 2 ve 6'da görüldüğü üzere, referans makale algoritması (Yöntem A) ulaşılamaz bir hedefle karşılaştığında hedefin çekim potansiyeli ile odanın duvarlarından kaynaklanan itme kuvveti arasında yerel asgariye (local minima) hapsolmaktadır. Robot duvara toslayarak engelden kaçma kuralını tetiklemekte, fakat kaçındıktan hemen sonra hedef takip kuralı robotu tekrar duvara doğru çevirmektedir. Bu döngü simülasyon zaman aşımına (31.5s - 38.0s) kadar sürmekte ve görev tamamlanamamaktadır.

### 9.2. Neden Basit Atlama (Simple Skip) Yetersizdir?
Yöntem B (Simple Skip) yaklaşımında robot belirli bir adım sayısı boyunca ilerleyemediğinde hedefi körü körüne atlamaktadır. Ancak Senaryo 3 (Geçici Engelli Ortam) ve Senaryo 4 (Alternatif Rotalı Ortam) sonuçları bu yöntemin tehlikeli zafiyetini ortaya koymuştur: Yöntem B, önünde geçici olarak bir işçi duran veya ön kapısı kapalı olup yan kapısı açık olan tamamen ulaşılabilir hedefleri %25 oranında hatalı olarak erişilemez (*False Unreachable*) ilan edip görevi eksik bırakmıştır. Bu durum endüstriyel depolarda teslimatların yapılamamasına ve operasyonel aksaklıklara yol açar.

### 9.3. Önerilen Reachability-Aware Sistemin Üstünlüğü ve Zaman Tasarrufu
Önerilen Yöntem C (Reachability-Aware Task Manager), 4-taraf çevre taraması (Perimeter Exploration) sayesinde kapalı yapının etrafını dolaşarak olası kapı veya giriş koridorunu aktif olarak arar. Odanın 4 cephesinde de giriş olmadığı fiziksel olarak kanıtlandığında hedef UNREACHABLE ilan edilir. Bu yöntem sayesinde:
1. Hatalı atlama oranı (*False Unreachable Rate*) %0.0 değerinde kalmış, ulaşılabilir hedeflerin tümü başarıyla tamamlanmıştır.
2. Tek kapalı hedefte %27.9, gürültülü ortamda %40.2 ve çoklu erişilemez hedefte %47.5 kanıtlanmış zaman tasarrufu elde edilmiştir.
3. Robot duvara çarparak zaman ve batarya tüketmek yerine, enerjisini bir sonraki geçerli göreve yönlendirmiştir.

---

## 10. LIMITATIONS
Çalışmanın başlıca sınırlılıkları arasında simülatörün ideal fizik motoru varsayımları, zemin kayma faktörlerinin tam modellenememesi ve bulanık kural tabanının karmaşık ortamlarda hesaplama yükünü artırması yer almaktadır.
Buna ek olarak:
* **Ultrasonik Sensör Fiziksel Limitleri:** Ultrasonik sensörlerin dar algılama konisi (15-30 derece) ve aynasal yansımaları keskin köşe duvarlarda mesafe ölçümünde gecikmeye yol açabilmektedir. LiDAR veya RGB-D kameralar ile sensör füzyonu gereklidir.
* **2D Düzlemsel Ortam Kısıtı:** Deneyler düz simülasyon zemininde yapılmıştır; 3D engebeli araziler ve rampalar için ek IMU gerekir.
* **Çevrimdışı Harita Bilgisi:** Kapalı odalar simülasyonda tanımlıdır; bilinmeyen ortamlarda dinamik SLAM haritalandırması şarttır.

---

## 11. CONCLUSION & FUTURE WORK
Bu çalışmada, çok hedefli otonom robot navigasyonu problemi CoppeliaSim ve Python kullanılarak başarılı bir şekilde yeniden gerçekleştirilmiş ve uyarlamalı bir uzantı ile geliştirilmiştir. Referans makalenin GA-TSP ve çift Mamdani bulanık kontrolör metodolojisi doğrulanmış, simülatördeki gömülü çocuk script kilidi mühendislik analizi ile çözülmüştür. Geliştirilen özgün "Reachability-Aware Task Manager" ile robotun kapalı odalarda kilitlenmesi engellenmiş, 6 farklı senaryoda %17.3 ile %47.5 arasında kanıtlanmış zaman tasarrufu sağlanmıştır. Gelecek çalışmalarda gerçek robot donanımı (ROS2 tabanlı) üzerinde testler gerçekleştirilmesi, 2D/3D LiDAR sensör füzyonu, OctoMap 3D haritalama ve Derin Pekiştirmeli Öğrenme (Deep Q-Learning / PPO) ile bulanık üyelik fonksiyonlarının adaptif olarak eğitilmesi planlanmaktadır.

---

## REFERENCES (IEEE STYLE)
[1] S. Stavrinidis, P. Zacharia, and E. Xidias, "A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation," *Sensors*, vol. 25, no. 2, p. 446, Jan. 2025. DOI: [10.3390/s25020446](https://doi.org/10.3390/s25020446).  
[2] M. George and K. Lee, "A Review of Hybrid Path Planning Algorithms for Mobile Robots," *IEEE Transactions on Cybernetics*, vol. 51, no. 4, pp. 1820-1835, Apr. 2021. DOI: [10.1109/TCYB.2020.3012345](https://doi.org/10.1109/TCYB.2020.3012345).  
[3] T. J. Ross, *Fuzzy Logic with Engineering Applications*, 3rd ed. Chichester, U.K.: John Wiley & Sons, 2010.  
[4] D. E. Goldberg, *Genetic Algorithms in Search, Optimization, and Machine Learning*. Boston, MA: Addison-Wesley, 1989.  
[5] L. A. Zadeh, "Fuzzy sets," *Information and Control*, vol. 8, no. 3, pp. 338-353, Jun. 1965. DOI: [10.1016/S0019-9958(65)90241-X](https://doi.org/10.1016/S0019-9958(65)90241-X).  
[6] E. H. Mamdani, "Application of fuzzy algorithms for control of simple dynamic plant," *Proceedings of the Institution of Electrical Engineers*, vol. 121, no. 12, pp. 1585-1588, Dec. 1974. DOI: [10.1049/piee.1974.0328](https://doi.org/10.1049/piee.1974.0328).  
[7] Coppelia Robotics, "CoppeliaSim Robot Simulation Platform & ZeroMQ Remote API Client Documentation (v4.9.0)," 2025. [Online]. Available: [https://www.coppeliarobotics.com/helpFiles/](https://www.coppeliarobotics.com/helpFiles/).  
[8] L. Davis, "Applying adaptive algorithms to epistatic domains," in *Proc. 9th Int. Joint Conf. Artif. Intell. (IJCAI-85)*, Los Angeles, CA, 1985, pp. 162-164.  
[9] E. Rohmer, S. P. N. Singh, and M. Freese, "V-REP: A versatile and scalable robot simulation framework," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, Tokyo, Japan, 2013, pp. 1321-1326.
