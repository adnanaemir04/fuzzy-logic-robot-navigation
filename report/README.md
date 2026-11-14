# Proje Raporu (Project Report)

Bu klasör, çalışmanın akademik ve teknik proje raporunu içermektedir.

## 📄 Dosya Listesi

* **[report.pdf](report.pdf)**: 7 sayfadan oluşan akademik formatlı detaylı teknik proje raporu.

## 📑 Rapor İçeriği ve Bölüm Yapısı

1. **Bölüm 1 (Giriş ve Araştırma Motivasyonu):** Otonom mobil robotlarda çoklu hedef dolaşımı, kombinatoryal zorluklar ve yerel kilitlenme problemleri.
2. **Bölüm 2 (Referans Makale Bilgisi ve Analizi):** MDPI Sensors 2025 makalesinin (Stavrinidis vd.) detaylı künyesi, açık erişim bilgileri ve metodolojik analizi.
3. **Bölüm 3 (Problem Tanımı ve Matematiksel Modelleme):** Diferansiyel sürüş mobil robot kinematiği (Pioneer P3-DX), TSP mesafe optimizasyon formülasyonu, konum ve açı hata metrikleri.
4. **Bölüm 4 (Çift Bulanık Mantık Denetleyici Mimarisi):** Hedef Takip (15 Mamdani kuralı, e_pos, e_head) ve Engelden Kaçınma (25 Mamdani kuralı, d_left, d_right) denetleyicileri, üyelik fonksiyonları, Centroid durulaştırması.
5. **Bölüm 5 (Reproduction: Kök Neden Analizi ve Motor Kilidi Çözümü):** CoppeliaSim Pioneer P3-DX gömülü Braitenberg Lua Child Script çakışmasının tespiti ve `_disable_child_script()` Python ZeroMQ çözümü.
6. **Bölüm 6 (Genetik Algoritma Tabanlı Global Rota Optimizasyonu):** Order Crossover (OX) ve Inversion Mutation ile 12 hedefli TSP çözümü.
7. **Bölüm 7 (Özgün Katkı: Erişilebilirlik Farkındalıklı Görev Yöneticisi):** 6 durumlu durum makinesi (PENDING, REACHABLE, TEMPORARILY_BLOCKED, UNCERTAIN, UNREACHABLE, COMPLETED), 4-taraf çevre taraması ve durgunluk analizi.
8. **Bölüm 8 (Deneysel Sonuçlar ve Karşılaştırmalı Değerlendirme):** 3 Yöntem x 6 Senaryo sayısal sonuç tablosu, %17.3 - %47.5 süre tasarrufu analizi ve grafiksel karşılaştırmalar.
9. **Bölüm 9 (Sınırlılıklar ve Gelecek Çalışmalar):** Ultrasonik sensör sınırları, 2D varsayımı, LiDAR/RGB-D füzyonu, SLAM ve Derin RL entegrasyonu.
10. **Bölüm 10 (Proje Repository Dizin Yapısı):** Repository organizasyonu ve dosya hiyerarşisi.
11. **Bölüm 11 (Kaynaklar):** IEEE/APA formatında tam bilimsel kaynakça.
