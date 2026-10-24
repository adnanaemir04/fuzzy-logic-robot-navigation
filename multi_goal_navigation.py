"""
multi_goal_navigation.py
------------------------
"A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation"
(Stavros Stavrinidis, Paraskevi Zacharia, Elias K. Xidias - Sensors 2025)

Tam Otonom Çoklu Hedef Navigasyon Sistemi:
1. Başlangıç noktasından ve hedef noktalar kümesinden başlar.
2. GA (Genetik Algoritma) ile en optimum hedef ziyaret sırasını belirler.
3. Her hedef için:
   - Engel yoksa -> Fuzzy Tracking Controller (Hedefe yönelme)
   - 1 metre içinde engel algılandığında -> Fuzzy Obstacle Avoidance Controller (En yüksek öncelik)
4. Hedefe yaklaşıldığında hedef tamamlanır ve sıradakine geçilir.
5. Tüm hedefler bitince robot güvenli bir şekilde başlangıç noktasına (depot) dönüp durur.
6. Sonuçları CSV dosyasına kaydeder.
"""

import csv
import math
import time
import numpy as np

from fuzzy_controller import FuzzyTrackingController, FuzzyObstacleAvoidanceController
from ga_tsp import GeneticTSP
from robot_interface import PioneerRobot


def normalize_angle(angle):
    """Açıyı [-pi, pi] aralığında sınırlar."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def run_navigation():
    print("=== Pioneer P3-DX Çoklu Hedef Bulanık Navigasyon Başlatılıyor ===", flush=True)
    robot = PioneerRobot()

    # Denetleyicileri oluştur
    tracking_ctrl = FuzzyTrackingController(max_wheel_speed=2.5)
    avoidance_ctrl = FuzzyObstacleAvoidanceController(safe_distance=1.0)

    # Simülasyonu başlat
    robot.start_simulation()
    time.sleep(0.5)

    try:
        # Robot başlangıç pozisyonunu al
        start_x, start_y, start_yaw = robot.get_pose()
        print(f"Robot Başlangıç Konumu: X={start_x:.2f}, Y={start_y:.2f}, Yaw={math.degrees(start_yaw):.1f}°", flush=True)

        # Makaledeki gibi çoklu hedef noktaları
        goal_points = [
            [start_x + 1.5, start_y + 0.8],
            [start_x + 1.5, start_y - 0.8],
            [start_x + 0.5, start_y - 0.5],
        ]

        # 1. Aşama: GA ile En Kısa TSP Sırasını Çöz
        print("\n[GA] Genetik Algoritma ile optimum hedef sırası hesaplanıyor...", flush=True)
        ga = GeneticTSP(start_pos=[start_x, start_y], goal_points=goal_points, pop_size=30, generations=30)
        best_tour, est_distance = ga.solve()

        ordered_goals = [goal_points[i] for i in best_tour]
        # Son hedef olarak tekrar başlangıç noktasına (depot) dönüş
        ordered_goals.append([start_x, start_y])

        print(f"[GA] Optimum Rota Belirlendi: Ziyaret Sırası = {best_tour} -> Başlangıç", flush=True)
        print(f"[GA] Tahmini Toplam Rota Mesafesi: {est_distance:.2f} m\n", flush=True)

        # Metrik loglama için liste
        trajectory_log = []
        step_count = 0
        total_path_length = 0.0
        prev_x, prev_y = start_x, start_y
        goal_reach_threshold = 0.28  # metre (Pioneer robot yarıçapı ile uyumlu)

        start_time = time.time()

        for goal_idx, target in enumerate(ordered_goals):
            target_x, target_y = target
            is_depot = (goal_idx == len(ordered_goals) - 1)
            target_name = "BAŞLANGIÇ (Depot)" if is_depot else f"Hedef #{goal_idx + 1} ({target_x:.2f}, {target_y:.2f})"
            print(f"\n>>> YENİ GÖREV: {target_name} yönüne ilerleniyor...", flush=True)

            goal_reached = False

            for step in range(150):
                cur_x, cur_y, cur_yaw = robot.get_pose()

                # Yol uzunluğunu topla
                step_dist = math.hypot(cur_x - prev_x, cur_y - prev_y)
                total_path_length += step_dist
                prev_x, prev_y = cur_x, cur_y

                # Hedefe olan mesafe (Position Error) ve Açı Hatası (Heading Error)
                dx = target_x - cur_x
                dy = target_y - cur_y
                pos_error = math.hypot(dx, dy)

                target_angle = math.atan2(dy, dx)
                heading_error = normalize_angle(target_angle - cur_yaw)

                # Sensörleri oku
                d_left, d_right = robot.read_front_sensors()

                # Engel kontrolü (Makale: d < 1.0 m ise engel kaçınma aktif)
                is_obstacle = avoidance_ctrl.is_obstacle_detected(d_left, d_right)

                if is_obstacle:
                    # Engelden Kaçınma Bulanık Denetleyicisi
                    v_l, v_r = avoidance_ctrl.evaluate(d_left, d_right)
                    mode = "OBSTACLE_AVOIDANCE"
                else:
                    # Hedef Takip Bulanık Denetleyicisi
                    v_l, v_r = tracking_ctrl.evaluate(pos_error, heading_error)
                    mode = "TRACKING"

                # Hızları motora gönder
                robot.set_wheel_velocities(v_l, v_r)

                # Metrik kaydet
                step_count += 1
                trajectory_log.append({
                    "step": step_count,
                    "x": cur_x,
                    "y": cur_y,
                    "yaw": cur_yaw,
                    "pos_error": pos_error,
                    "heading_error": heading_error,
                    "d_left": d_left,
                    "d_right": d_right,
                    "mode": mode,
                    "v_left": v_l,
                    "v_right": v_r,
                    "target_idx": goal_idx
                })

                if step % 10 == 0:
                    print(f"[{mode:<18}] Kalan: {pos_error:.2f} m | Açı: {math.degrees(heading_error):+.1f}° | Hızlar: L={v_l:.2f}, R={v_r:.2f}", flush=True)

                # Hedefe ulaşma kontrolü
                if pos_error < goal_reach_threshold:
                    goal_reached = True
                    print(f">>> TEBRİKLER: {target_name} noktasına ulaşıldı! (Hata: {pos_error:.3f} m)", flush=True)
                    break

                time.sleep(0.08)

            if not goal_reached:
                print(f">>> Zaman aşımı: {target_name} tamamlanamadı, sonraki hedefe geçiliyor...", flush=True)

        total_time = time.time() - start_time
        robot.stop_motors()

        print("\n================ GÖREV ÖZETİ ================", flush=True)
        print(f"Toplam Hedef Noktası   : {len(ordered_goals)}", flush=True)
        print(f"Toplam Harcanan Süre   : {total_time:.2f} saniye", flush=True)
        print(f"Toplam Kat Edilen Yol  : {total_path_length:.2f} metre", flush=True)
        print(f"Toplam Adım Sayısı     : {step_count}", flush=True)
        print("=============================================", flush=True)

        # CSV dosyasına kaydet
        csv_filename = "navigation_results.csv"
        if trajectory_log:
            with open(csv_filename, mode="w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=trajectory_log[0].keys())
                writer.writeheader()
                writer.writerows(trajectory_log)
            print(f"Deney sonuçları '{csv_filename}' dosyasına kaydedildi.", flush=True)

    finally:
        robot.stop_simulation()
        print("Simülasyon durduruldu.", flush=True)


if __name__ == "__main__":
    run_navigation()
