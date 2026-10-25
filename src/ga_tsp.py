"""
ga_tsp.py
---------
"A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation"
(Stavros Stavrinidis, Paraskevi Zacharia, Elias K. Xidias - Sensors 2025)

Makaledeki Gezgin Satıcı Problemi (TSP) Optimizasyonu:
- Çoklu hedef noktaları (Goal points / Inspection points) için en kısa ve verimli
  ziyaret sırasını belirleyen Genetik Algoritma (GA).
- Kromozom kodlaması: Tamsayı hedef sıra dizisi (Order Crossover - OX ve Inversion Mutation).
"""

import math
import random
import numpy as np


class GeneticTSP:
    def __init__(self, start_pos, goal_points, pop_size=50, generations=100, mutation_rate=0.15, obstacle_segments=None):
        """
        start_pos: Robotun başlangıç konumu [x, y]
        goal_points: Ziyaret edilecek hedef noktaların listesi [[x1, y1], [x2, y2], ...]
        obstacle_segments: [((x1, y1), (x2, y2)), ...] duvar çizgi segmentleri
        """
        self.start_pos = np.array(start_pos)
        self.goal_points = np.array(goal_points)
        self.num_goals = len(goal_points)
        self.pop_size = pop_size
        self.generations = generations
        self.mutation_rate = mutation_rate
        self.obstacle_segments = obstacle_segments or []

    def _ccw(self, A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def _intersect(self, A, B, C, D):
        """AB doğru parçası ile CD doğru parçası kesişiyor mu kontrol eder."""
        return self._ccw(A, C, D) != self._ccw(B, C, D) and self._ccw(A, B, C) != self._ccw(A, B, D)

    def _euclidean_distance(self, p1, p2):
        base_d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
        # Eğer bu iki nokta arasındaki düz hat bir duvarı delip geçiyorsa yüksek ceza ekle:
        for seg in self.obstacle_segments:
            if self._intersect(p1, p2, seg[0], seg[1]):
                base_d += 12.0  # Duvarı doğrudan delip geçme cezası (dolambaç maliyeti)
        return base_d

    def calculate_total_distance(self, tour):
        """
        Belirli bir ziyaret sırasının toplam yol uzunluğunu hesaplar.
        Rota: Başlangıç -> Goal_1 -> Goal_2 -> ... -> Goal_N -> Başlangıç (depot)
        """
        dist = 0.0
        current_pos = self.start_pos
        for goal_idx in tour:
            target = self.goal_points[goal_idx]
            dist += self._euclidean_distance(current_pos, target)
            current_pos = target
        # Başlangıca dönüş
        dist += self._euclidean_distance(current_pos, self.start_pos)
        return dist

    def _order_crossover(self, parent1, parent2):
        """Order Crossover (OX) operatörü."""
        size = len(parent1)
        a, b = sorted(random.sample(range(size), 2))
        child = [-1] * size
        child[a:b + 1] = parent1[a:b + 1]

        p2_idx = 0
        for i in range(size):
            if child[i] == -1:
                while parent2[p2_idx] in child:
                    p2_idx += 1
                child[i] = parent2[p2_idx]
        return child

    def _inversion_mutation(self, tour):
        """Tersine çevirme (Inversion) mutasyonu."""
        if random.random() < self.mutation_rate:
            a, b = sorted(random.sample(range(len(tour)), 2))
            tour[a:b + 1] = reversed(tour[a:b + 1])
        return tour

    def _two_opt(self, tour):
        """2-Opt yerel arama ile rotadaki çapraz kesişimleri ve fazlalıkları çözer."""
        improved = True
        best_tour = tour.copy()
        best_dist = self.calculate_total_distance(best_tour)

        while improved:
            improved = False
            for i in range(len(best_tour) - 1):
                for j in range(i + 1, len(best_tour)):
                    candidate = best_tour[:i] + best_tour[i:j + 1][::-1] + best_tour[j + 1:]
                    cand_dist = self.calculate_total_distance(candidate)
                    if cand_dist < best_dist - 1e-4:
                        best_tour = candidate
                        best_dist = cand_dist
                        improved = True
                        break
                if improved:
                    break
        return best_tour, best_dist

    def solve(self):
        """
        Gezgin Satıcı Problemi (TSP) Optimizasyonu:
        1. Nearest Neighbor Sezgisel Yaklaşımı ile anında başlangıç turu üretilir.
        2. GA küresel arama ve 2-Opt yerel arama ile rota çaprazlıkları tamamen çözülüp
           kesin en kısa rota milisaniyeler içinde elde edilir.
        """
        if self.num_goals <= 2:
            return list(range(self.num_goals)), self.calculate_total_distance(list(range(self.num_goals)))

        # 1. Aşama: En Yakın Komşu (Nearest Neighbor) ile güçlü bir başlangıç rotası oluştur
        unvisited = set(range(self.num_goals))
        current_pos = self.start_pos
        nn_tour = []
        while unvisited:
            next_idx = min(unvisited, key=lambda i: self._euclidean_distance(current_pos, self.goal_points[i]))
            nn_tour.append(next_idx)
            unvisited.remove(next_idx)
            current_pos = self.goal_points[next_idx]

        # 2. Aşama: Genetik Algoritma Popülasyonunu NN turu ile aşıla (Memetic GA)
        population = [nn_tour.copy()]
        # Kalan popülasyonu çeşitlendir
        for _ in range(self.pop_size - 1):
            ind = nn_tour.copy()
            # Rastgele 2 noktayı yer değiştir
            i, j = random.sample(range(self.num_goals), 2)
            ind[i], ind[j] = ind[j], ind[i]
            population.append(ind)

        best_tour = nn_tour.copy()
        best_distance = self.calculate_total_distance(best_tour)

        for gen in range(self.generations):
            distances = [self.calculate_total_distance(ind) for ind in population]
            for i, d in enumerate(distances):
                if d < best_distance:
                    best_distance = d
                    best_tour = population[i].copy()

            fitnesses = [1.0 / (d + 1e-6) for d in distances]
            total_fitness = sum(fitnesses)
            probs = [f / total_fitness for f in fitnesses]

            new_population = [best_tour.copy()]

            while len(new_population) < self.pop_size:
                p1, p2 = random.choices(population, weights=probs, k=2)
                child = self._order_crossover(p1, p2)
                child = self._inversion_mutation(child)
                new_population.append(child)

            population = new_population

        # 3. Aşama: 2-Opt yerel arama ile rotadaki tüm kesişimleri sıfırla
        refined_tour, refined_dist = self._two_opt(best_tour)
        return refined_tour, refined_dist
