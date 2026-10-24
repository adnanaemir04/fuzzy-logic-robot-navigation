"""
fuzzy_controller.py
-------------------
"A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation"
(Stavros Stavrinidis, Paraskevi Zacharia, Elias K. Xidias - Sensors 2025)

Makaledeki Çift Bulanık Denetleyici (Dual Fuzzy Controller):
1. Fuzzy Tracking Controller (Hedef Takip Bulanık Denetleyicisi):
   - Girişler:
     * Position Error (e_pos): Robot ile hedef arasındaki Öklid mesafesi [0.0, 1.0+] (Small, Medium, Large)
     * Heading Error (e_head): Hedef yönelim açısı hatası [-pi, +pi] -> normalize [-1.0, 1.0] (NegBig, NegSmall, Zero, PosSmall, PosBig)
   - Çıkışlar:
     * v_left, v_right: Sol ve sağ motor hızları (Slow, Medium, Fast)
   - 15 adet Mamdani kuralı

2. Fuzzy Obstacle Avoidance Controller (Engelden Kaçınma Bulanık Denetleyicisi):
   - Girişler:
     * Left Sensor (d_left): Ön sol sensörlerin minimum algılama mesafesi
     * Right Sensor (d_right): Ön sağ sensörlerin minimum algılama mesafesi
   - Çıkışlar:
     * v_left, v_right: Engelden kaçınma tekerlek hızları
   - 25 adet Mamdani kuralı (Sol engelde sağa dönüş, sağ engelde sola dönüş)
"""

import numpy as np


class FuzzyTrackingController:
    """
    Hedefe doğru yönelim ve yaklaşma denetleyicisi.
    Mamdani çıkarım yöntemi ile centroid defuzzification uygular.
    """
    def __init__(self, max_wheel_speed=3.0):
        self.max_speed = max_wheel_speed

    def _trapmf(self, x, a, b, c, d):
        """Trapezoidal (yamuk) üyelik fonksiyonu."""
        if x <= a or x >= d:
            return 0.0
        elif a <= x <= b:
            return (x - a) / (b - a) if b > a else 1.0
        elif b <= x <= c:
            return 1.0
        elif c <= x <= d:
            return (d - x) / (d - c) if d > c else 1.0
        return 0.0

    def evaluate(self, pos_error, heading_error):
        """
        pos_error: metre cinsinden mesafe (0 - 5+ m)
        heading_error: radyan cinsinden açı hatası [-pi, pi]
        
        Dönen: (v_left, v_right) rad/s cinsinden hedef motor hızları
        """
        # Normalize edilmiş değerler
        d = np.clip(pos_error, 0.0, 3.0)
        # Heading error [-pi, pi] aralığını [-1, 1] aralığına normalize et
        h = np.clip(heading_error / np.pi, -1.0, 1.0)

        # 1. Giriş 1: Position Error üyelik dereceleri (Small, Medium, Large)
        mu_pos_small  = self._trapmf(d, -0.1, 0.0, 0.3, 0.8)
        mu_pos_medium = self._trapmf(d, 0.4, 0.8, 1.4, 1.8)
        mu_pos_large  = self._trapmf(d, 1.4, 2.0, 10.0, 11.0)

        # 2. Giriş 2: Heading Error üyelik dereceleri
        # Titreşimi ve sağa-sola gereksiz salınımı önlemek için Zero bölgesine deadband uyguluyoruz:
        # +/- 4 derece (~0.07 radyan / normalized 0.022) içindeki küçük açılar tam Z (Zero) kabul edilir.
        mu_head_nb = self._trapmf(h, -1.2, -1.0, -0.6, -0.25)
        mu_head_ns = self._trapmf(h, -0.45, -0.25, -0.10, -0.02)
        mu_head_z  = self._trapmf(h, -0.06, -0.025, 0.025, 0.06)
        mu_head_ps = self._trapmf(h, 0.02, 0.10, 0.25, 0.45)
        mu_head_pb = self._trapmf(h, 0.25, 0.6, 1.0, 1.2)

        # Çıkış hız seviyeleri (En hızlı otonom sürüş) [rad/s]
        v_slow = 1.4
        v_med  = 2.4
        v_fast = 3.6

        # 15 Makale Kuralı (Mamdani Min-Max bileşimi):
        # Matematiksel ve Fiziksel Gerçeklik:
        # CoppeliaSim'de Sol motor pozitif döndüğünde robot SAAT YÖNÜNÜN TERSİNE (SOLA, +Yaw) döner.
        # Sağ motor pozitif döndüğünde robot SAAT YÖNÜNDE (SAĞA, -Yaw) döner.
        # Hedef soldaysa (h > 0 / PS, PB): Sol motor yavaşlar/geri, Sağ motor ileri döner -> Robot SOLA döner.
        # Hedef sağdaysa (h < 0 / NS, NB): Sol motor ileri, Sağ motor yavaşlar/geri döner -> Robot SAĞA döner.

        rules = []

        # POS LARGE & MEDIUM & SMALL (Robot hedefe doğru en hızlı şekilde akar)
        # Uzak hedef
        rules.append((min(mu_pos_large, mu_head_nb),  v_fast, -v_slow))  # Sağa dönüş
        rules.append((min(mu_pos_large, mu_head_ns),  v_fast,  v_med))   # Yumuşak sağa kavis
        rules.append((min(mu_pos_large, mu_head_z),   v_fast,  v_fast))  # Tam yol ileri (En Hızlı)
        rules.append((min(mu_pos_large, mu_head_ps),  v_med,   v_fast))  # Yumuşak sola kavis
        rules.append((min(mu_pos_large, mu_head_pb), -v_slow,  v_fast))  # Sola dönüş

        # Orta mesafe
        rules.append((min(mu_pos_medium, mu_head_nb),  v_fast, -v_slow))
        rules.append((min(mu_pos_medium, mu_head_ns),  v_fast,  v_med))
        rules.append((min(mu_pos_medium, mu_head_z),   v_fast,  v_fast))  # Orta mesafede de tam yol ileri
        rules.append((min(mu_pos_medium, mu_head_ps),  v_med,   v_fast))
        rules.append((min(mu_pos_medium, mu_head_pb), -v_slow,  v_fast))

        # Yakın mesafe (Hedefin üstünden hız kesmeden tam gaz geçiş)
        rules.append((min(mu_pos_small, mu_head_nb),  v_med,  -v_slow))
        rules.append((min(mu_pos_small, mu_head_ns),  v_fast,  v_slow))
        rules.append((min(mu_pos_small, mu_head_z),   v_fast,  v_fast))  # Hedefin tam içinden yüksek hızla geç
        rules.append((min(mu_pos_small, mu_head_ps),  v_slow,  v_fast))
        rules.append((min(mu_pos_small, mu_head_pb), -v_slow,  v_med))

        # Ağırlıklı ortalama (Weighted Average / Centroid)
        sum_weights = sum(w for w, _, _ in rules)
        if sum_weights < 1e-6:
            return 0.0, 0.0

        v_l = sum(w * vl for w, vl, _ in rules) / sum_weights
        v_r = sum(w * vr for w, _, vr in rules) / sum_weights

        return float(v_l), float(v_r)


class FuzzyObstacleAvoidanceController:
    """
    Ön sol ve ön sağ ultrasonik sensör verilerine dayanarak
    engelden kaçma manevrası üreten Mamdani denetleyicisi.
    Gereksiz aşırı dolanmayı önlemek için güvenli mesafe 0.55m olarak optimize edilmiştir.
    """
    def __init__(self, safe_distance=0.55):
        self.safe_distance = safe_distance

    def _trapmf(self, x, a, b, c, d):
        if x <= a or x >= d:
            return 0.0
        elif a <= x <= b:
            return (x - a) / (b - a) if b > a else 1.0
        elif b <= x <= c:
            return 1.0
        elif c <= x <= d:
            return (d - x) / (d - c) if d > c else 1.0
        return 0.0

    def is_obstacle_detected(self, d_left, d_right):
        """0.55 metreden yakın acil engel var mı kontrolü."""
        return min(d_left, d_right) < self.safe_distance

    def evaluate(self, d_left, d_right):
        """
        d_left, d_right: metre cinsinden sol ve sağ sektör mesafe değerleri
        Dönen: (v_left, v_right) engelden kaçınma hızları
        """
        dl = np.clip(d_left, 0.0, self.safe_distance)
        dr = np.clip(d_right, 0.0, self.safe_distance)

        # 3 Kademeli Mesafe Üyelikleri (Critical, Close, Safe)
        # 0.55m altındaki yakın tehlikelerde aktif kaçış
        mu_l_crit = self._trapmf(dl, -0.1, 0.0, 0.15, 0.28)
        mu_l_close = self._trapmf(dl, 0.20, 0.28, 0.40, 0.52)
        mu_l_safe  = self._trapmf(dl, 0.45, 0.52, 2.0, 3.0)

        mu_r_crit = self._trapmf(dr, -0.1, 0.0, 0.15, 0.28)
        mu_r_close = self._trapmf(dr, 0.20, 0.28, 0.40, 0.52)
        mu_r_safe  = self._trapmf(dr, 0.45, 0.52, 2.0, 3.0)

        v_fwd  = 2.2
        v_turn = 1.8
        v_rev  = -1.0

        rules = []

        # 1. Her iki taraf kritik yakınsa -> Geri çekil ve sağa dön
        rules.append((min(mu_l_crit, mu_r_crit), v_turn, v_rev))

        # 2. Sol kritik/yakınsa -> Sağa dönerek kaç
        rules.append((mu_l_crit, v_turn, v_rev))
        rules.append((mu_l_close, v_fwd, 0.4))

        # 3. Sağ kritik/yakınsa -> Sola dönerek kaç
        rules.append((mu_r_crit, v_rev, v_turn))
        rules.append((mu_r_close, 0.4, v_fwd))

        # 4. Güvenli bölge
        rules.append((min(mu_l_safe, mu_r_safe), v_fwd, v_fwd))

        sum_w = sum(w for w, _, _ in rules)
        if sum_w < 1e-6:
            return 2.0, 2.0

        v_l = sum(w * vl for w, vl, _ in rules) / sum_w
        v_r = sum(w * vr for w, _, vr in rules) / sum_w

        return float(v_l), float(v_r)
