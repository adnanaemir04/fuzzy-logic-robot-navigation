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
    def __init__(self, start_pos, goal_points, pop_size=50, generations=100, mutation_rate=0.15):
        """
        start_pos: Robotun başlangıç konumu [x, y]
        goal_points: Ziyaret edilecek hedef noktaların listesi [[x1, y1], [x2, y2], ...]
        """
        self.start_pos = np.array(start_pos)
        self.goal_points = np.array(goal_points)
        self.num_goals = len(goal_points)
        self.pop_size = pop_size
        self.generations = generations
        self.mutation_rate = mutation_rate

    def _euclidean_distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

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

    def solve(self):
        """
        Genetik algoritmayı çalıştırır ve en uygun rota sırasını döndürür.
        """
        if self.num_goals <= 2:
            return list(range(self.num_goals)), self.calculate_total_distance(list(range(self.num_goals)))

        # 1. Başlangıç Popülasyonu
        population = []
        for _ in range(self.pop_size):
            individual = list(range(self.num_goals))
            random.shuffle(individual)
            population.append(individual)

        best_tour = None
        best_distance = float('inf')

        for gen in range(self.generations):
            # Uygunluk (Fitness) hesaplama (Ters mesafe)
            distances = [self.calculate_total_distance(ind) for ind in population]
            for i, d in enumerate(distances):
                if d < best_distance:
                    best_distance = d
                    best_tour = population[i].copy()

            # Rulet tekerleği seçimi için fitness
            fitnesses = [1.0 / (d + 1e-6) for d in distances]
            total_fitness = sum(fitnesses)
            probs = [f / total_fitness for f in fitnesses]

            # Yeni nesil oluşturma
            new_population = [best_tour.copy()]  # Elitizm: En iyiyi doğrudan koru

            while len(new_population) < self.pop_size:
                p1, p2 = random.choices(population, weights=probs, k=2)
                child = self._order_crossover(p1, p2)
                child = self._inversion_mutation(child)
                new_population.append(child)

            population = new_population

        return best_tour, best_distance
