# Pioneer P3-DX Otonom Coklu Hedef Bulanik Navigasyon Projesi
Bu proje, **"A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation"** (Sensors 2025, Stavros Stavrinidis, Paraskevi Zacharia, Elias K. Xidias) bilimsel calismasindaki yontemlerin Python ve CoppeliaSim uzerinde yeniden gerceklestirilmesidir.

## Proje Bilesenleri ve Mimarisi
1. **`fuzzy_controller.py`**:
   - **Fuzzy Tracking Controller:** Konum ve yonelim acisi hatasina gore 15 Mamdani kurali ile sol ve sag tekerlek hizlarini hesaplar.
   - **Fuzzy Obstacle Avoidance Controller:** On ultrasonik sensorlerden 0.85m altinda engel algilandiginda devreye girerek 25 Mamdani kurali ile guvenli kacis manevrasi uretir.
2. **`ga_tsp.py`**:
   - Coklu hedef noktalari (TSP) icin en kisa rota siralamasini optimize eden Genetik Algoritma (Order Crossover + Inversion Mutation).
3. **`robot_interface.py`**:
   - CoppeliaSim ZeroMQ Remote API baglantisi, robot pozisyon ve sensor okuma, gomulu cakisici Lua scriptlerini otomatik notralize etme.
4. **`scene_visualizer.py`**:
   - CoppeliaSim sahnesinde canli kirmizi iz cizgisi (Trajectory Trail), hedeflerde yesil kureler ve rotada kirmizi silindir engeller olusturur.
5. **`multi_goal_navigation.py`**:
   - Tam otonom coklu hedef gorevini canli olarak baslatir ve metrikleri `navigation_results.csv` dosyasina kaydeder.
6. **`benchmark_experiments.py`**:
   - 5 farkli bilimsel senaryoda (Engelsiz, Tek Engelli, Coklu Engelli, Dar Gecis, GA'siz) karsilastirmali deneyleri otomatik kosturur ve `benchmark_comparison.png` grafigini uretir.
7. **`plot_results.py`**:
   - Makale Figur 30 formatinda konum hatasi, 2D rota, yonelim hatasi ve motor hiz profillerini gorsellestirir (`navigation_performance.png`).

## Benchmark Sonuclari (5 Senaryo Ozeti)
| Senaryo | Basari Orani (%) | Toplam Yol (m) | Toplam Sure (s) | Carpisma Sayisi | Ortalama Hedef Hatasi (m) |
|---|:---:|:---:|:---:|:---:|:---:|
| 1. Engel Bulunmayan Ortam | %100.0 | 5.97 m | 14.25 s | 0 | 0.312 m |
| 2. Tek Engelli Ortam | %100.0 | 6.53 m | 16.08 s | 0 | 0.311 m |
| 3. Coklu Engelli Ortam | %100.0 | 6.92 m | 15.88 s | 0 | 0.312 m |
| 4. Dar Gecis Senaryosu | %75.0 | 7.01 m | 20.93 s | 0 | 0.312 m |
| 5. GA Olmadan (Duz Sira) | %100.0 | 6.59 m | 19.07 s | 0 | 0.311 m |

## Calistirma
```bash
# Canli tekil coklu hedef navigasyonunu baslatmak icin:
python multi_goal_navigation.py

# 5 senaryolu benchmark testlerini kosturmak icin:
python benchmark_experiments.py

# Performans grafigini yeniden uretmek icin:
python plot_results.py
```
