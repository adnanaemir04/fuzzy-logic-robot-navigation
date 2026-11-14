# Kaynaklar (References) & Akademik Literatür

Bu çalışmada referans alınan temel bilimsel yayınlar, simülasyon dokümantasyonları ve klasik literatür kaynakları aşağıda akademik standartlara uygun olarak sunulmuştur.

---

## 1. Temel Referans Makale (Original Paper)

* **Makale Başlığı:** *A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation*  
* **Yazarlar:** Stavros Stavrinidis, Paraskevi Zacharia, Elias K. Xidias  
* **Akademik Editörler:** Stefano Quer, Qirong Tang  
* **Dergi:** *Sensors* (MDPI), Cilt 25, Sayı 2, Madde 446 (Ocak 2025)  
* **DOI:** [10.3390/s25020446](https://doi.org/10.3390/s25020446)  
* **PMCID:** [PMC11769363](https://pmc.ncbi.nlm.nih.gov/articles/PMC11769363/)  
* **Açık Erişim (MDPI):** [https://www.mdpi.com/1424-8220/25/2/446](https://www.mdpi.com/1424-8220/25/2/446)  

### Makaleden Alınan ve Yeniden Üretilen (Reproduction) Temel İlkeler:
1. **Çift Mamdani Bulanık Mantık Denetleyicisi (Dual Fuzzy Logic Controller):**
   - **Hedef Takip Denetleyicisi (Tracking):** Konum hatası ($e_{pos}$) ve yönelim açısı hatası ($e_{head}$) girdileriyle 15 Mamdani kuralı üzerinden $v_L, v_R$ tekerlek hızlarını belirler.
   - **Engelden Kaçınma Denetleyicisi (Avoidance):** Ön 6 ultrasonik sensörün sol ($d_{left}$) ve sağ ($d_{right}$) sektör olarak gruplanması ve 25 Mamdani kuralı üzerinden engelden kaçış manevrası üretir.
2. **Kombinatoryal Hedef Rota Optimizasyonu (GA-TSP):**
   - Ziyaret edilecek 12 hedef noktasının diziliminde Genetik Algoritma (Sıralı Çaprazlama / Order Crossover ve Ters Çevirme / Inversion Mutation) kullanımı.
3. **Simülasyon Doğrulaması:**
   - Pioneer P3-DX mobil robot platformu ve CoppeliaSim üzerinde telemetri doğrulaması.

```bibtex
@article{stavrinidis2025fuzzy,
  title={A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation},
  author={Stavrinidis, Stavros and Zacharia, Paraskevi and Xidias, Elias K.},
  journal={Sensors},
  volume={25},
  number={2},
  pages={446},
  year={2025},
  publisher={MDPI},
  doi={10.3390/s25020446}
}
```

---

## 2. Destekleyici ve İlgili Bilimsel Kaynaklar

1. **Bulanık Mantık Kuramı:**
   - Zadeh, L. A. (1965). *Fuzzy sets.* Information and Control, 8(3), 338–353. [https://doi.org/10.1016/S0019-9958(65)90241-X](https://doi.org/10.1016/S0019-9958(65)90241-X)
   - Mamdani, E. H. (1974). *Application of fuzzy algorithms for control of simple dynamic plant.* Proceedings of the Institution of Electrical Engineers, 121(12), 1585–1588. [https://doi.org/10.1049/piee.1974.0328](https://doi.org/10.1049/piee.1974.0328)

2. **Genetik Algoritmalar ve Gezgin Satıcı Problemi (TSP):**
   - Goldberg, D. E. (1989). *Genetic Algorithms in Search, Optimization, and Machine Learning.* Addison-Wesley Longman Publishing Co., Inc.
   - Davis, L. (1985). *Applying adaptive algorithms to epistatic domains.* In Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI-85), pp. 162–164. (Order Crossover / OX temeli).

3. **Robotik Simülasyon Platformu ve Donanım:**
   - Rohmer, E., Singh, S. P., & Freese, M. (2013). *V-REP: A versatile and scalable robot simulation framework.* In 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems (pp. 1321–1326). (Güncel adıyla CoppeliaSim).
   - Coppelia Robotics (2025). *CoppeliaSim Robot Simulation Platform & ZeroMQ remote API Documentation (v4.9.0).* [https://www.coppeliarobotics.com/helpFiles/](https://www.coppeliarobotics.com/helpFiles/)
