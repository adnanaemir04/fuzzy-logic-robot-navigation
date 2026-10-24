# Referanslar (References)

Bu çalışmada kullanılan temel bilimsel makale ve teknik kaynaklar aşağıda listelenmiştir:

## 1. Temel Referans Makale
* **Başlık:** A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation
* **Yazarlar:** Stavros Stavrinidis, Paraskevi Zacharia, Elias K. Xidias
* **Akademik Editörler:** Stefano Quer, Qirong Tang
* **Dergi:** *Sensors* (MDPI), 2025, 25(2), 446
* **DOI:** [10.3390/s25020446](https://doi.org/10.3390/s25020446)
* **PMCID:** [PMC11769363](https://pmc.ncbi.nlm.nih.gov/articles/PMC11769363/)
* **Açık Erişim:** [MDPI Sensors Makale Bağlantısı](https://www.mdpi.com/1424-8220/25/2/446)

### Makaleden Alınan Temel Metodolojiler:
1. **Çift Bulanık Denetleyici (Dual Fuzzy Logic Controller):**
   - Hedef Takip Denetleyicisi: Mesafe hatası ($e_{pos}$) ve yönelim açısı hatası ($e_{head}$) girdileriyle 15 Mamdani kuralı.
   - Engelden Kaçınma Denetleyicisi: Ön ultrasonik sensörlerin sol ($d_{left}$) ve sağ ($d_{right}$) sektör olarak gruplanması ve 25 Mamdani kuralı.
2. **Kombinatoryal Optimizasyon (TSP):**
   - Ziyaret edilecek hedef noktalarının sıralanmasında Genetik Algoritma (GA) kullanımı (Order Crossover ve Inversion Mutation).
3. **Simülasyon Doğrulaması:**
   - Pioneer P3-DX mobil robotu ile CoppeliaSim üzerinde test edilmesi.

---

## 2. Destekleyici ve İlgili Kaynaklar
1. **Coppelia Robotics:**
   - CoppeliaSim Robot Simulation Platform & ZeroMQ remote API Client Documentation (v4.9.0).
   - [https://www.coppeliarobotics.com/helpFiles/](https://www.coppeliarobotics.com/helpFiles/)
2. **Goldberg, D. E. (1989):**
   - *Genetic Algorithms in Search, Optimization, and Machine Learning.* Addison-Wesley.
3. **Zadeh, L. A. (1965):**
   - *Fuzzy sets.* Information and Control, 8(3), 338-353.
4. **Mamdani, E. H. (1974):**
   - *Application of fuzzy algorithms for control of simple dynamic plant.* Proceedings of the Institution of Electrical Engineers, 121(12), 1585-1588.
