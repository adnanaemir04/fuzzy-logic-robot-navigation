# Çoklu Hedef Otonom Robot Navigasyonu için Bulanık Mantık ve Genetik Algoritma Tabanlı Kontrol Stratejisi

> **Proje Türü:** Bilimsel Makale Yeniden Üretimi (Reproduction) ve Özgün Geliştirme  
> **Öğrenci / Geliştirici:** Emir Kırılmış (`adnanemir04`)  
> **E-posta:** emirkirimis5101@gmail.com  
> **Simülasyon Ortamı:** CoppeliaSim Edu v4.9.0  
> **Robot Platformu:** Pioneer P3-DX Diferansiyel Sürüşlü Mobil Robot  
> **Programlama Dili & IDE:** Python 3.13, Antigravity IDE  

---

## 1. Referans Makale Bilgisi
- **Makale Başlığı:** *A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation*  
- **Yazarlar:** Stavros Stavrinidis, Paraskevi Zacharia, Elias K. Xidias  
- **Akademik Editörler:** Stefano Quer, Qirong Tang  
- **Dergi:** *Sensors* (MDPI), Cilt 25, Sayı 2, Madde 446 (Ocak 2025)  
- **DOI:** [10.3390/s25020446](https://doi.org/10.3390/s25020446) | **PMCID:** PMC11769363  
- **Açık Erişim URL:** [https://www.mdpi.com/1424-8220/25/2/446](https://www.mdpi.com/1424-8220/25/2/446)

---

## 2. Problem Tanımı
Mobil robotların karmaşık ve engellerle dolu iç ortamlarda (depo, fabrika sahası, muayene görevleri vb.) önceden belirlenmiş birden fazla hedef noktayı en kısa toplam mesafeyle dolaşarak üsse geri dönmesi gerekmektedir. Bu problem:
1. **Hedef Ziyaret Sıralaması (Kombinatoryal NP-hard TSP):** Hedeflerin hangi sırayla ziyaret edileceğinin optimizasyonu.
2. **Statik ve Dinamik Çarpışmasız Navigasyon:** Rota üzerinde aniden beliren statik engeller (kutular, duvarlar) ve dinamik engeller (hareketli yayalar/işçiler) karşısında gerçek zamanlı ve kararlı yön değiştirme ihtiyacı.

---

## 3. Kullanılan Yöntem ve Mimari
Proje iki ana katmandan oluşmaktadır:
1. **Global Rota Optimizasyonu Katmanı (`src/ga_tsp.py`):**
   - Belirlenen hedef noktaları Gezgin Satıcı Problemi (Traveling Salesperson Problem - TSP) olarak modellenir.
   - **Genetik Algoritma (GA):** Sıralı çaprazlama (Order Crossover - OX) ve Ters Çevirme (Inversion Mutation) operatörleri ile en kısa toplam mesafeyi veren hedef rotasını bulur.
2. **Yerel Karar ve Sürüş Katmanı (`src/fuzzy_controller.py`):**
   - **Hedef Takip Bulanık Denetleyicisi (Fuzzy Tracking Controller):** Robotun hedef ile arasındaki mesafe hatası ($e_{pos}$) ve yönelim açısı hatasını ($e_{head}$) işleyerek tekerlek hızlarını ($v_L, v_R$) yönlendirir (15 Mamdani kuralı).
   - **Engelden Kaçınma Bulanık Denetleyicisi (Fuzzy Obstacle Avoidance Controller):** Ön 6 ultrasonik sensörden türetilen sol ($d_{left}$) ve sağ ($d_{right}$) sektör mesafe değerlerine göre en yüksek öncelikle engelden uzaklaşma manevrası üretir (25 Mamdani kuralı).

---

## 4. Bulanık Mantık Sistem Yapısı (Fuzzy System Architecture)

### 4.1. Hedef Takip Bulanık Denetleyicisi (Tracking)
- **Giriş 1: Konum Hatası ($e_{pos}$):** [0, 3+] m aralığında 3 adet yamuk (trapezoidal) üyelik fonksiyonu:
  - *Small* (Küçük: [0.0 - 0.8m]), *Medium* (Orta: [0.4 - 1.8m]), *Large* (Büyük: [1.4m+])
- **Giriş 2: Yönelim Hatası ($e_{head}$):** [-pi, +pi] aralığında 5 adet yamuk üyelik fonksiyonu:
  - *NB* (Negatif Büyük), *NS* (Negatif Küçük), *Z* (Sıfır), *PS* (Pozitif Küçük), *PB* (Pozitif Büyük)
- **Çıkışlar:** $v_{left}, v_{right}$ (Sol ve Sağ Motor Hızları: [Slow: 0.5 rad/s, Med: 1.5 rad/s, Fast: 2.5 rad/s])
- **Kural Tabanı:** 15 adet Mamdani kuralı, durulaştırma için *Centroid (Ağırlık Merkezi)* yöntemi.

### 4.2. Engelden Kaçınma Bulanık Denetleyicisi (Obstacle Avoidance)
- **Girişler:** $d_{left}$ ve $d_{right}$ (Sol ve Sağ Ön Ultrasonik Sektör Mesafeleri)
- **Üyelik Kümeleri:** *Very Close (VC)*, *Close (C)*, *Medium (M)*, *Far (F)*, *Safe (S)*
- **Eşik Değeri:** Sensörler 0.85m altında nesne algıladığında en yüksek öncelikle engelden kaçma modu devreye girer.
- **Kural Tabanı:** Sol engel baskınken sağ motor hızlanıp sol motor yavaşlar (sağa kaçış); sağ engel baskınken sol motor hızlanır (sola kaçış); kafa kafaya engelde robot geri çekilerek döner.

---

## 5. Proje Dizin Yapısı (Repository Directory Structure)
```text
fuzzy-logic-project/
├── README.md                           # Kapsamlı akademik proje kılavuzu
├── LICENSE                             # MIT Lisans belgesi
├── requirements.txt                    # Bağımlılıklar ve kesin sürümleri
├── .gitignore                          # Git takip dışı dosyalar
│
├── src/                                # Temel kaynak kodlar ve modüller
│   ├── reachability_manager.py         # [ÖZGÜN KATKI] Erişilebilirlik Farkındalıklı Görev Yöneticisi
│   ├── fuzzy_controller.py             # Çift Bulanık Denetleyici (Tracking & Avoidance)
│   ├── ga_tsp.py                       # Genetik Algoritma TSP Çözücü
│   ├── robot_interface.py              # CoppeliaSim Remote API ve Robot Donanım Arayüzü
│   └── scene_visualizer.py             # 4 Duvarlı Kapalı Oda, Canlı iz çizgisi, hedefler, engeller
│
├── data/                               # Sayısal veri setleri ve kaydedilen loglar
│   ├── reachability_task_results.csv   # [ÖZGÜN] Görev yöneticisi hedef durum kayıtları
│   ├── navigation_results.csv          # Çoklu hedef sürüş telemetri kayıtları
│   └── benchmark_results.csv           # 5 senaryolu temel benchmark ölçümleri
│
├── experiments/                        # Deney betikleri ve senaryo kodları
│   ├── baseline/                       # [TEMEL / MAKALENİN BİREBİR YENİDEN ÜRETİMİ]
│   │   ├── multi_goal_navigation.py    # Makale tabanlı standart çoklu hedef navigasyonu
│   │   ├── benchmark_experiments.py    # 5 Temel senaryo testi
│   │   ├── stop_test.py                # Motor durdurma kök neden analizi ve testi
│   │   ├── move_robot.py               # Temel hareket doğrulama testi
│   │   ├── sensor-test.py              # Ultrasonik sensör okuma testi
│   │   └── test-connection.py          # ZeroMQ API bağlantı testi
│   │
│   └── proposed/                       # [ÖZGÜN GELİŞTİRME & ÖNERİLEN YÖNTEM]
│       ├── reachability_navigation.py  # 4 Duvarlı Kapalı Oda & Reachability Manager Navigasyonu
│       ├── comparative_evaluation.py   # 3 Yöntem (A, B, C) & 6 Senaryo Karşılaştırmalı Benchmark
│       ├── multi_goal_navigation.py    # Gelişmiş sönümlü fuzzy yaklaşım navigasyonu
│       └── plot_results.py             # Makale formatında performans grafik çizici
│
├── tests/                              # Birim ve entegrasyon testleri
│   └── test_reachability_manager.py    # Durum makinesi ve topoloji testleri
│
├── results/                            # Çıktılar, grafikler ve tablolar
│   ├── figures/                        # Çıktı grafikleri ve görsel sonuçlar
│   │   ├── reachability_comparison.png # [ÖZGÜN] 3 Yöntemli görev süresi ve israf karşılaştırması
│   │   ├── navigation_performance.png  # Rota, konum hatası, hız profilleri
│   │   └── benchmark_comparison.png    # Senaryolar arası yol ve süre karşılaştırması
│   └── tables/                         # Özet metrik tabloları (CSV kopyaları)
│       └── reachability_benchmark_comparison.csv
```

---

## 6. Kurulum ve Gereksinimler

### 6.1. Sistem Gereksinimleri
- İşletim Sistemi: Windows 10/11
- Python Sürümü: Python 3.10+ (Test edilen sürüm: Python 3.13)
- Simülatör: CoppeliaSim Edu v4.9.0

### 6.2. Kütüphanelerin Kurulumu
```powershell
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

---

## 7. Çalıştırma Adımları

1. **CoppeliaSim'i Açın:**
   - CoppeliaSim uygulamasını başlatın (`Pioneer P3-DX` robotu sahnede hazır olsun).
2. **Özgün Katkı: Reachability-Aware Navigation (4 Duvarlı Kapalı Oda):**
   ```powershell
   .venv\Scripts\python.exe experiments/proposed/reachability_navigation.py
   ```
   *Ekranda:* En üstteki kırmızı kutu yerine 4 duvardan oluşan tamamen kapalı oda ve içine erişilemez hedef yerleştirilir. Robot duvara çarpmadan çevresini inceler, hedefi `UNREACHABLE` olarak sınıflandırıp atlar ve diğer hedefleri başarıyla tamamlar.
3. **Özgün Katkı: 3 Yöntemli Karşılaştırmalı Benchmark (6 Senaryo):**
   ```powershell
   .venv\Scripts\python.exe experiments/proposed/comparative_evaluation.py
   ```
4. **Birim Testlerini Çalıştırma:**
   ```powershell
   .venv\Scripts\python.exe -m unittest discover -s tests
   ```
5. **Temel Yöntem (Baseline / Makale Yeniden Üretimi):**
   ```powershell
   .venv\Scripts\python.exe experiments/baseline/multi_goal_navigation.py
   ```

---

## 8. Özgün Katkı: Reachability-Aware Task Manager

Mevcut makale tabanlı mobil navigasyon sistemleri, hedeflerin topolojik veya fiziksel olarak ulaşılamaz olduğu durumlarda duvara toslayarak ya da sonsuz engelden kaçma döngüsüne girerek görevi kilitlemektedir.

Bu projede geliştirilen **Reachability-Aware Task Manager** (`src/reachability_manager.py`):
1. **6 Kademeli Hedef Durum Makinesi:** `PENDING`, `REACHABLE`, `TEMPORARILY_BLOCKED`, `UNCERTAIN`, `UNREACHABLE`, `COMPLETED`.
2. **Topolojik ve Fiziksel Doğrulama:** 4 duvarla tamamen kapatılmış odalardaki hedefleri topolojik analiz ve teğetsel çevre keşfi (`Perimeter Exploration`) ile tespit eder.
3. **Durgunluk Analizi (Stagnation Detection):** Robotun engel karşısında salınımını veya net ilerleme kaydedemediğini tespit eder.
4. **Zaman ve Enerji Tasarrufu:** Erişilemeyen hedefi körü körüne denemek yerine güvenli biçimde terk eder, görevi durdurmaz ve kalan ulaşılabilir hedefleri tamamlar.

### 8.1. 3 Yöntemli Karşılaştırmalı Sonuçlar (6 Senaryo)

- **Yöntem A (Baseline):** Mevcut makale algoritması (erişilemezlik farkındalığı yok).
- **Yöntem B (Simple Skip):** Sabit deneme sayısından sonra körü körüne hedef atlama.
- **Yöntem C (Reachability-Aware):** Harita bağlantısallığı, ilerleme takibi ve durum makinesiyle önerilen yöntem.

| Senaryo | Yöntem A Süre (s) | Yöntem B Süre (s) | Yöntem C (Önerilen) Süre (s) | Yöntem C Zaman Tasarrufu |
|---|:---:|:---:|:---:|:---:|
| **1. Tümü Ulaşılabilir Ortam** | 26.00 s | 26.00 s | **26.00 s** | Referans |
| **2. Tek Hedef 4 Duvarla Kapalı** | 31.50 s (Takıldı) | 31.50 s | **22.70 s** | **%27.9 Daha Hızlı** |
| **3. Geçici Engelli Ortam** | 33.50 s | 29.50 s (Yanlış Atladı) | **27.70 s** | **%17.3 Daha Hızlı** |
| **4. Alternatif Rotalı Ortam** | 33.50 s | 29.50 s (Yanlış Atladı) | **27.70 s** | **%17.3 Daha Hızlı** |
| **5. Sensör Gürültülü Ortam** | 38.00 s (Takıldı) | 31.50 s | **22.70 s** | **%40.2 Daha Hızlı** |
| **6. Çoklu Erişilemez Hedef** | 31.50 s (Takıldı) | 37.00 s | **19.40 s** | **%47.5 Daha Hızlı** |

---

## 9. Başlangıç Kök Neden Analizi (Baseline / Reproduction İlk Aşaması)
Projenin başlangıcında `experiments/baseline/stop_test.py` dosyasında robot motorlarına `0.0` hedef hız komutu gönderilmesine rağmen gerçek hızın `2.000 rad/s` olarak kaldığı ve robotun durmadığı tespit edilmiştir.
- **Kök Neden:** CoppeliaSim'in varsayılan `PioneerP3DX` modelinde gömülü olan Braitenberg Lua Child Script'i her simülasyon adımında `sim.setJointTargetVelocity(motor, 2.0)` komutunu tekrar çalıştırarak Python komutlarını ezmekteydi.
- **Çözüm:** `src/robot_interface.py` içerisine entegre edilen `_disable_child_script()` fonksiyonu ile sahnedeki gömülü script dinamik olarak tespit edilip nötralize edilmiş ve robotun fiziksel hızı `0.000 rad/s` değerine indirilerek tam kontrol sağlanmıştır.

---

## 10. Sınırlılıklar ve Gelecek Çalışmalar
- Ultrasonik sensörlerin dar algılama konisi sebebiyle çok keskin köşe duvarlarda mesafe geç algılanabilmektedir; LiDAR veya RGB-D kamera füzyonu ile bu durum iyileştirilebilir.
- Dinamik harita çıkarma (SLAM) entegrasyonu ile kapalı alanlar simülasyon esnasında anlık oktree / occupancy grid olarak modellenebilir.

---

## 11. Güvenlik Notu
Bu repository içerisinde hiçbir kişisel erişim anahtarı (API token, password, private key) bulunmamaktadır. Tüm kimlik doğrulama gereksinimleri yerel ortam değişkenleri veya kullanıcı yetkilendirmesi üzerinden yönetilmektedir.

---

## 12. Lisans
Bu proje [MIT Lisansı](LICENSE) altında lisanslanmıştır.
