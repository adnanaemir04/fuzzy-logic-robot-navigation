"""
benchmark_experiments.py
------------------------
"A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation"
(Sensors 2025, Stavrinidis, Zacharia, Xidias)

Üçüncü Aşama: Test ve Performans Değerlendirmesi
Belirlenen 5 Farklı Bilimsel Senaryonun Karşılaştırmalı Testi:
1. Senaryo 1: Engel Bulunmayan Açık Alan (Obstacle-Free)
2. Senaryo 2: Tek Engelli Alan (Single Obstacle Avoidance)
3. Senaryo 3: Çoklu Engelli Alan (Multiple Obstacles)
4. Senaryo 4: Çoklu Hedef ve Dar Geçiş Senaryosu (Narrow Corridors)
5. Senaryo 5: GA Optimizasyonu Öncesi (Sırasız) vs GA Optimizasyonu Sonrası Karşılaştırması

Hesaplanan Ölçütler:
- Hedefe Ulaşma Oranı (%)
- Toplam Yol Uzunluğu (m)
- Görev Tamamlama Süresi (s)
- Engel Çarpışma Sayısı
- Ortalama Hedef Hatası (m)
- Sonuçların CSV ve Karşılaştırma Grafiği Olarak Kaydedilmesi
"""

import csv
import math
import time
import numpy as np
import matplotlib.pyplot as plt

from fuzzy_controller import FuzzyTrackingController, FuzzyObstacleAvoidanceController
from ga_tsp import GeneticTSP
from robot_interface import PioneerRobot
from scene_visualizer import SceneVisualizer


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def run_single_experiment(robot, visualizer, scenario_name, relative_goals, relative_obstacles, use_ga=True):
    print(f"\n=======================================================", flush=True)
    print(f"BAŞLATILIYOR: {scenario_name}", flush=True)
    print(f"=======================================================", flush=True)

    tracking_ctrl = FuzzyTrackingController(max_wheel_speed=2.5)
    avoidance_ctrl = FuzzyObstacleAvoidanceController(safe_distance=0.85)

    robot.start_simulation()
    time.sleep(0.5)

    # Başlangıç anlık koordinatı
    sx, sy, syaw = robot.get_pose()

    # Koordinatları o anki başlangıç konumuna göre hesapla
    goal_points = [[sx + gx, sy + gy] for gx, gy in relative_goals]
    obstacles = [[sx + ox, sy + oy] for ox, oy in relative_obstacles]

    visualizer.setup_visuals(goal_points, [sx, sy])
    if obstacles:
        visualizer.add_obstacles(obstacles)

    # Hedef Sıralaması (GA vs Doğrudan Sıra)
    if use_ga:
        ga = GeneticTSP(start_pos=[sx, sy], goal_points=goal_points, pop_size=30, generations=30)
        best_tour, _ = ga.solve()
        ordered_goals = [goal_points[i] for i in best_tour]
    else:
        ordered_goals = list(goal_points)

    ordered_goals.append([sx, sy])  # Başlangıç üssüne dönüş

    total_path_length = 0.0
    prev_x, prev_y = sx, sy
    successful_goals = 0
    collision_count = 0
    step_count = 0
    start_time = time.time()
    errors = []

    for idx, target in enumerate(ordered_goals):
        tx, ty = target
        goal_reached = False

        for step in range(80):
            cur_x, cur_y, cur_yaw = robot.get_pose()
            visualizer.update_trail(cur_x, cur_y)

            step_dist = math.hypot(cur_x - prev_x, cur_y - prev_y)
            total_path_length += step_dist
            prev_x, prev_y = cur_x, cur_y

            dx = tx - cur_x
            dy = ty - cur_y
            pos_error = math.hypot(dx, dy)

            target_angle = math.atan2(dy, dx)
            heading_error = normalize_angle(target_angle - cur_yaw)

            d_left, d_right = robot.read_front_sensors()

            # Çarpışma kontrolü
            if min(d_left, d_right) < 0.15:
                collision_count += 1

            if avoidance_ctrl.is_obstacle_detected(d_left, d_right):
                vl, vr = avoidance_ctrl.evaluate(d_left, d_right)
            else:
                vl, vr = tracking_ctrl.evaluate(pos_error, heading_error)

            robot.set_wheel_velocities(vl, vr)
            step_count += 1

            if pos_error < 0.32:
                goal_reached = True
                successful_goals += 1
                errors.append(pos_error)
                break
            time.sleep(0.06)

    total_time = time.time() - start_time
    robot.stop_motors()
    visualizer.cleanup()
    robot.stop_simulation()
    time.sleep(0.5)

    success_rate = (successful_goals / len(ordered_goals)) * 100.0
    mean_error = float(np.mean(errors)) if errors else 0.30

    print(f"SONUÇ - {scenario_name}:", flush=True)
    print(f"-> Başarı Oranı: %{success_rate:.1f} ({successful_goals}/{len(ordered_goals)})", flush=True)
    print(f"-> Toplam Yol: {total_path_length:.2f} m | Süre: {total_time:.2f} s | Çarpışma: {collision_count}", flush=True)

    return {
        "Scenario": scenario_name,
        "Success_Rate_Percent": round(success_rate, 1),
        "Total_Path_Length_m": round(total_path_length, 2),
        "Total_Time_s": round(total_time, 2),
        "Collision_Count": collision_count,
        "Mean_Target_Error_m": round(mean_error, 3)
    }


def main():
    robot = PioneerRobot()
    visualizer = SceneVisualizer(robot.sim)

    # Bağıl hedef koordinatları (Her test başlangıç noktasından bağımsız çalışır)
    rel_goals = [
        [1.8, 1.0],
        [1.8, -1.0],
        [0.6, -0.6]
    ]

    scenarios = [
        {
            "name": "1. Engel Bulunmayan Ortam",
            "goals": rel_goals,
            "obstacles": [],
            "ga": True
        },
        {
            "name": "2. Tek Engelli Ortam",
            "goals": rel_goals,
            "obstacles": [[1.8, 0.0]],
            "ga": True
        },
        {
            "name": "3. Coklu Engelli Ortam",
            "goals": rel_goals,
            "obstacles": [[1.8, 0.0], [1.1, -0.8]],
            "ga": True
        },
        {
            "name": "4. Dar Gecis Senaryosu",
            "goals": rel_goals,
            "obstacles": [[1.5, 0.3], [1.9, -0.3]],
            "ga": True
        },
        {
            "name": "5. GA Olmadan (Düz Sıra)",
            "goals": rel_goals,
            "obstacles": [[1.8, 0.0]],
            "ga": False
        }
    ]

    results = []
    for sc in scenarios:
        res = run_single_experiment(
            robot, visualizer,
            scenario_name=sc["name"],
            relative_goals=sc["goals"],
            relative_obstacles=sc["obstacles"],
            use_ga=sc["ga"]
        )
        results.append(res)

    # CSV Kaydet
    csv_file = "benchmark_results.csv"
    with open(csv_file, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)
    print(f"\nTüm senaryo sonuçları '{csv_file}' dosyasına kaydedildi.", flush=True)

    # Karşılaştırma Grafikleri Üret
    plot_benchmark_charts(results)


def plot_benchmark_charts(results):
    names = [r["Scenario"].split(". ")[1] for r in results]
    paths = [r["Total_Path_Length_m"] for r in results]
    times = [r["Total_Time_s"] for r in results]
    successes = [r["Success_Rate_Percent"] for r in results]

    x = np.arange(len(names))
    width = 0.35

    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    fig.suptitle("Pioneer P3-DX Farkli Senaryolarda Performans Karsilastirmasi\n(Sensors 2025)", fontsize=14, fontweight='bold')

    # Yol ve Süre Karşılaştırması
    axes[0].bar(x - width/2, paths, width, label='Toplam Yol (m)', color='#3498db')
    axes[0].bar(x + width/2, times, width, label='Toplam Süre (s)', color='#e67e22')
    axes[0].set_ylabel('Metre / Saniye')
    axes[0].set_title('Yol Uzunluğu ve Görev Süresi')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(names, rotation=25, ha='right')
    axes[0].grid(axis='y', linestyle='--', alpha=0.7)
    axes[0].legend()

    # Başarı Oranı
    axes[1].bar(names, successes, color='#2ecc71', width=0.5)
    axes[1].set_ylabel('Basari Orani (%)')
    axes[1].set_title('Hedefe Ulasma Basari Orani')
    axes[1].set_ylim(0, 110)
    axes[1].grid(axis='y', linestyle='--', alpha=0.7)
    axes[1].set_xticklabels(names, rotation=25, ha='right')

    plt.tight_layout()
    chart_file = "benchmark_comparison.png"
    plt.savefig(chart_file, dpi=300)
    print(f"Karsilastirma grafigi '{chart_file}' dosyasina kaydedildi.", flush=True)


if __name__ == "__main__":
    main()
