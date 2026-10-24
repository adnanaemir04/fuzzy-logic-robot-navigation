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

---

## 2. Problem Tanımı
Mobil robotların karmaşık ve engellerle dolu iç ortamlarda (depo, fabrika sahası, arama-kurtarma vb.) önceden belirlenmiş birden fazla hedef noktayı (muayene veya görev istasyonları) en kısa toplam mesafeyle dolaşarak üsse geri dönmesi gerekmektedir. Bu problem:
1. **Hedef Ziyaret Sıralaması (Kombinatoryal NP-hard TSP):** Hedeflerin hangi sırayla ziyaret edileceğinin optimizasyonu.
2. **Statik ve Dinamik Çarpışmasız Navigasyon:** Rota üzerinde aniden beliren statik engeller (kutular, duvarlar) ve dinamik engeller (hareketli yayalar/işçiler) karşısında gerçek zamanlı ve kararlı yön değiştirme ihtiyacı.

---

## 3. Kullanılan Yöntem ve Mimari
Proje iki ana katmandan oluşmaktadır:
1. **Global Rota Optimizasyonu Katmanı (GA-TSP):**
   - Belirlenen hedef noktaları Gezgin Satıcı Problemi (Traveling Salesperson Problem - TSP) olarak modellenir.
   - **Genetik Algoritma (GA):** Sıralı çaprazlama (Order Crossover - OX) ve Ters Çevirme (Inversion Mutation) operatörleri ile en kısa toplam mesafeyi veren hedef rotasını bulur.
2. **Yerel Karar ve Sürüş Katmanı (Dual Fuzzy Logic Controller):**
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

## 5. Proje Klasör Yapısı
```text
fuzzy-logic-project/
├── docs/                       # Bilimsel açıklamalar ve mimari dokümanlar
├── results/                    # Deney çıktıları, CSV tabloları ve grafikler
│   ├── benchmark_results.csv
│   ├── benchmark_comparison.png
│   ├── navigation_results.csv
│   └── navigation_performance.png
├── fuzzy_controller.py         # Çift Bulanık Denetleyici (Tracking & Avoidance)
├── ga_tsp.py                   # Genetik Algoritma TSP Rota Sıralayıcı
├── robot_interface.py          # Pioneer P3-DX CoppeliaSim ZMQ Arayüzü
├── scene_visualizer.py         # Canlı kırmızı iz çizgisi, küreler ve duvar engelleri
├── multi_goal_navigation.py    # 10 Hedefli + Duvar Engelli Ana Navigasyon Programı
├── benchmark_experiments.py    # 5 Farklı senaryo için otomatik benchmark testleri
├── plot_results.py             # Performans analiz grafiklerini üreten betik
├── stop_test.py                # Başlangıç motor durdurma ve kök neden doğrulama testi
├── requirements.txt            # Proje bağımlılıkları ve kesin sürümleri
├── .gitignore                  # Git takip dışı dosyalar
└── README.md                   # Kapsamlı proje kılavuzu
```

---

## 6. Kurulum ve Gereksinimler

### 6.1. Sistem Gereksinimleri
- Windows 10/11
- Python 3.10+ (Test edilen sürüm: Python 3.13)
- CoppeliaSim Edu v4.9.0 (veya v4.5+)

### 6.2. Python Paketlerinin Kurulumu
Sanal ortam oluşturup gerekli kütüphaneleri yükleyin:
```powershell
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

---

## 7. Çalıştırma Adımları

1. **CoppeliaSim'i Açın:**
   - CoppeliaSim programını başlatın.
   - Sahnede sol panelden `Models > Robots > Mobile > Pioneer p3dx` modelini sürükleyip sahneye bırakın (veya boş bir sahnede bekleyin; kod otomatik algılayacaktır).
2. **Ana Navigasyon Programını Çalıştırın:**
   ```powershell
   python multi_goal_navigation.py
   ```
   *Ekranda:* Sahneye 10 adet yeşil hedef küresi, kırmızı silindirler ve dikdörtgen prizma DUVAR engelleri eklenecek; robot arkasında canlı kırmızı iz bırakarak tüm hedefleri sırayla gezecektir.
3. **5 Senaryolu Karşılaştırmalı Deneyleri Koşturun:**
   ```powershell
   python benchmark_experiments.py
   ```
4. **Grafikleri Yeniden Üretmek İçin:**
   ```powershell
   python plot_results.py
   ```

---

## 8. Başlangıç Kök Neden Analizi (Baseline / Reproduction İlk Aşaması)
Projenin başlangıcında `stop_test.py` dosyasında robot motorlarına `0.0` hedef hız komutu gönderilmesine rağmen gerçek hızın `2.000 rad/s` olarak kaldığı ve robotun durmadığı tespit edilmiştir.
- **Kök Neden:** CoppeliaSim'in varsayılan `PioneerP3DX` modelinde gömülü olan Braitenberg Lua Child Script'i her simülasyon adımında `sim.setJointTargetVelocity(motor, 2.0)` komutunu tekrar çalıştırarak Python komutlarını ezmekteydi.
- **Çözüm:** `robot_interface.py` içerisine entegre edilen `_disable_child_script()` fonksiyonu ile sahnedeki gömülü script dinamik olarak tespit edilip nötralize edilmiş ve robotun fiziksel hızı `0.000 rad/s` değerine indirilerek tam kontrol sağlanmıştır.

---

## 9. Deneysel Sonuçlar ve Karşılaştırma Tabloları

### 9.1. 5 Farklı Senaryoda Benchmark Sonuçları
`benchmark_experiments.py` tarafından üretilen karşılaştırma tablosu:

| Senaryo | Hedefe Ulaşma Oranı | Toplam Yol (m) | Görev Süresi (s) | Engel Çarpışması | Ortalama Hata (m) |
|---|:---:|:---:|:---:|:---:|:---:|
| **1. Engel Bulunmayan Ortam** | **%100.0** (4/4) | **5.97 m** | **14.25 s** | **0** | **0.312 m** |
| **2. Tek Engelli Ortam** | **%100.0** (4/4) | **6.53 m** | **16.08 s** | **0** | **0.311 m** |
| **3. Çoklu Engelli Ortam** | **%100.0** (4/4) | **6.92 m** | **15.88 s** | **0** | **0.312 m** |
| **4. Dar Geçiş Senaryosu** | **%75.0** (3/4) | **7.01 m** | **20.93 s** | **0** | **0.312 m** |
| **5. GA Olmadan (Düz Sıra)** | **%100.0** (4/4) | **6.59 m** | **19.07 s** | **0** | **0.311 m** |

### 9.2. Özgün Geliştirme (Extended Environment)
- Orijinal makaledeki senaryoya ek olarak hedef sayısı **10'a çıkarılmış**, rotaya **koridor oluşturan duvar engelleri** ve **silindir engeller** eklenmiştir.
- Sahneye robotun izini canlı çizen **Trajectory Trail Visualizer** entegre edilmiştir.

---

## 10. Sınırlılıklar ve Gelecek Çalışmalar
- Ultrasonik sensörlerin dar açılı olması nedeniyle bazı keskin köşe duvarlarda mesafe geç algılanabilmektedir; LiDAR veya RGB-D kamera füzyonu ile bu durum iyileştirilebilir.
- Çok dar geçişlerde robotun geriye manevra kuralları geliştirilerek %100 dar geçiş başarısı hedeflenebilir.

---

## 11. Güvenlik Notu
Bu repository içerisinde hiçbir kişisel erişim anahtarı (API token, password, private key) bulunmamaktadır. Tüm kimlik doğrulama gereksinimleri yerel ortam değişkenleri veya kullanıcı yetkilendirmesi üzerinden yönetilmektedir.

---

## 12. Kaynaklar
1. Stavrinidis, S., Zacharia, P., & Xidias, E. K. (2025). *A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation.* Sensors, 25(2), 446. https://doi.org/10.3390/s25020446
2. Coppelia Robotics, *CoppeliaSim User Manual & ZeroMQ Remote API Documentation*, 2024.
3. Goldberg, D. E. (1989). *Genetic Algorithms in Search, Optimization, and Machine Learning.* Addison-Wesley.
