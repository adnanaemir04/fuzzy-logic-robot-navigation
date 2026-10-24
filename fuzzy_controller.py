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
        # NB: Negative Big, NS: Negative Small, Z: Zero, PS: Positive Small, PB: Positive Big
        mu_head_nb = self._trapmf(h, -1.2, -1.0, -0.6, -0.3)
        mu_head_ns = self._trapmf(h, -0.5, -0.3, -0.1, 0.0)
        mu_head_z  = self._trapmf(h, -0.15, -0.05, 0.05, 0.15)
        mu_head_ps = self._trapmf(h, 0.0, 0.1, 0.3, 0.5)
        mu_head_pb = self._trapmf(h, 0.3, 0.6, 1.0, 1.2)

        # Çıkış hız seviyeleri (Slow, Medium, Fast) [rad/s]
        v_slow = 0.5
        v_med  = 1.5
        v_fast = 2.5

        # 15 Makale Kuralı (Mamdani Min-Max bileşimi):
        # Matematiksel ve Fiziksel Gerçeklik:
        # CoppeliaSim'de Sol motor pozitif döndüğünde robot SAAT YÖNÜNÜN TERSİNE (SOLA, +Yaw) döner.
        # Sağ motor pozitif döndüğünde robot SAAT YÖNÜNDE (SAĞA, -Yaw) döner.
        # Hedef soldaysa (h > 0 / PS, PB): Sol motor yavaşlar/geri, Sağ motor ileri döner -> Robot SOLA döner.
        # Hedef sağdaysa (h < 0 / NS, NB): Sol motor ileri, Sağ motor yavaşlar/geri döner -> Robot SAĞA döner.

        rules = []

        # POS LARGE (Uzak hedef)
        rules.append((min(mu_pos_large, mu_head_nb),  v_fast, -v_med))  # Sert sağa dönüş
        rules.append((min(mu_pos_large, mu_head_ns),  v_fast,  v_slow))  # Hafif sağa dönüş
        rules.append((min(mu_pos_large, mu_head_z),   v_fast,  v_fast))  # Tam yol ileri
        rules.append((min(mu_pos_large, mu_head_ps),  v_slow,  v_fast))  # Hafif sola dönüş
        rules.append((min(mu_pos_large, mu_head_pb), -v_med,   v_fast))  # Sert sola dönüş

        # POS MEDIUM (Orta mesafe)
        rules.append((min(mu_pos_medium, mu_head_nb),  v_med, -v_slow))
        rules.append((min(mu_pos_medium, mu_head_ns),  v_med,  v_slow))
        rules.append((min(mu_pos_medium, mu_head_z),   v_med,  v_med))
        rules.append((min(mu_pos_medium, mu_head_ps),  v_slow, v_med))
        rules.append((min(mu_pos_medium, mu_head_pb), -v_slow, v_med))

        # POS SMALL (Hedefe varış / hassas yanaşma)
        rules.append((min(mu_pos_small, mu_head_nb),  v_slow, -v_slow))
        rules.append((min(mu_pos_small, mu_head_ns),  v_slow,  0.0))
        rules.append((min(mu_pos_small, mu_head_z),   v_slow,  v_slow))
        rules.append((min(mu_pos_small, mu_head_ps),  0.0,     v_slow))
        rules.append((min(mu_pos_small, mu_head_pb), -v_slow,  v_slow))

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
    """
    def __init__(self, safe_distance=1.0):
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
        """1 metreden yakın engel var mı kontrolü (Makale Section 6)."""
        return min(d_left, d_right) < self.safe_distance

    def evaluate(self, d_left, d_right):
        """
        d_left, d_right: metre cinsinden sol ve sağ sektör mesafe değerleri
        Dönen: (v_left, v_right) engelden kaçınma hızları
        """
        dl = np.clip(d_left, 0.0, self.safe_distance)
        dr = np.clip(d_right, 0.0, self.safe_distance)

        # 5 Seviyeli Mesafe Üyelikleri (Very Close, Close, Medium, Far, Safe)
        # VC: [0.0, 0.25], C: [0.2, 0.45], M: [0.4, 0.7], F: [0.65, 0.9], S: [0.85, 1.0+]
        def get_memberships(d):
            vc = self._trapmf(d, -0.1, 0.0, 0.15, 0.3)
            c  = self._trapmf(d, 0.2, 0.3, 0.45, 0.55)
            m  = self._trapmf(d, 0.45, 0.55, 0.7, 0.8)
            f  = self._trapmf(d, 0.7, 0.8, 0.9, 1.0)
            s  = self._trapmf(d, 0.85, 0.95, 2.0, 3.0)
            return vc, c, m, f, s

        l_vc, l_c, l_m, l_f, l_s = get_memberships(dl)
        r_vc, r_c, r_m, r_f, r_s = get_memberships(dr)

        v_fast = 2.0
        v_turn = 1.6
        v_rev  = -1.2

        rules = []

        # 1. Her iki taraf çok yakınsa (Dar boğaz veya kafa kafaya engel) -> Geri çekil ve sağa kaç
        rules.append((min(l_vc, r_vc), v_turn, v_rev))

        # 2. Sol çok yakın / yakınsa -> Sağa sert dönüş (Sol motor ileri, Sağ motor geri/yavaş)
        rules.append((l_vc, v_turn, v_rev))
        rules.append((l_c,  v_turn, 0.0))
        rules.append((l_m,  v_fast, 0.5))

        # 3. Sağ çok yakın / yakınsa -> Sola sert dönüş (Sol motor geri/yavaş, Sağ motor ileri)
        rules.append((r_vc, v_rev, v_turn))
        rules.append((r_c,  0.0,    v_turn))
        rules.append((r_m,  0.5,    v_fast))

        # 4. Sol ve sağ orta/uzaksa
        rules.append((min(l_f, r_f), v_fast, v_fast))
        rules.append((min(l_s, r_s), v_fast, v_fast))

        sum_w = sum(w for w, _, _ in rules)
        if sum_w < 1e-6:
            return 1.5, 1.5

        v_l = sum(w * vl for w, vl, _ in rules) / sum_w
        v_r = sum(w * vr for w, _, vr in rules) / sum_w

        return float(v_l), float(v_r)
