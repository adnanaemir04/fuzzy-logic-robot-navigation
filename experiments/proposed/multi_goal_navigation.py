"""
multi_goal_navigation.py
------------------------
"A Fuzzy Control Strategy for Multi-Goal Autonomous Robot Navigation"
(Stavros Stavrinidis, Paraskevi Zacharia, Elias K. Xidias - Sensors 2025)

Tam Otonom Çoklu Hedef Navigasyon Sistemi:
1. Başlangıç noktasından ve hedef noktalar kümesinden başlar.
2. GA (Genetik Algoritma) ile en optimum hedef ziyaret sırasını belirler.
3. CoppeliaSim sahnesinde:
   - Robotun arkasında CANLI KIRMIZI İZ (Trajectory Trail) çizer.
   - Hedef noktalarına YEŞİL KÜRELER (Goal Markers) yerleştirir.
   - Yol üzerine KIKMIZI SİLİNDİR ENGELLER (Obstacles) yerleştirir.
4. Engel algılandığında -> Fuzzy Obstacle Avoidance Controller (En yüksek öncelik)
5. Engel yoksa -> Fuzzy Tracking Controller (Hedefe yönelme)
6. Tüm hedefler bitince robot başlangıç noktasına (depot) dönüp durur.
7. Metrikleri CSV dosyasına kaydeder.
"""

import csv
import math
import os
import sys
import time
from pathlib import Path
import numpy as np

# src dizinini import arama yoluna ekle
current_dir = Path(__file__).resolve().parent
src_dir = current_dir.parents[1] / "src"
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

from fuzzy_controller import FuzzyTrackingController, FuzzyObstacleAvoidanceController
from ga_tsp import GeneticTSP
from robot_interface import PioneerRobot
from scene_visualizer import SceneVisualizer


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
    visualizer = SceneVisualizer(robot.sim)

    # Denetleyicileri oluştur
    tracking_ctrl = FuzzyTrackingController(max_wheel_speed=2.5)
    avoidance_ctrl = FuzzyObstacleAvoidanceController(safe_distance=0.8)

    # Robotun boşluğa düşmemesi için zemini 50x50 metre devasa boyuta genişlet
    visualizer.ensure_large_floor(target_size=50.0)

    # Simülasyonu başlat
    robot.start_simulation()
    time.sleep(0.5)

    try:
        # Robot başlangıç pozisyonunu al
        start_x, start_y, start_yaw = robot.get_pose()
        print(f"Robot Başlangıç Konumu: X={start_x:.2f}, Y={start_y:.2f}, Yaw={math.degrees(start_yaw):.1f}°", flush=True)

        # Sahneye eklenecek DUVAR ENGELLERİ:
        wall_configs = [
            {"pos": [start_x - 0.5, start_y + 0.4], "size": [0.15, 0.8, 0.5]},
            {"pos": [start_x + 0.7, start_y - 0.4], "size": [0.8, 0.15, 0.5]},
        ]

        # Zeminin içine serpiştirilmiş silindir engeller:
        obstacle_coords = [
            [start_x + 0.0, start_y - 0.6],
            [start_x - 0.5, start_y - 0.7],
            [start_x + 0.5, start_y + 0.8],
        ]

        # HER ÇALIŞTIRMADA TAMAMEN RASTGELE 10 HEDEF NOKTASI ÜRETİLİR
        # Hedefler zemin sınırları ([-1.7m, +1.7m]) içinde, duvarlarla ve birbirleriyle çakışmayacak şekilde dinamik oluşturulur:
        def generate_random_goals(num_targets=10):
            import random
            goals = []
            while len(goals) < num_targets:
                gx = round(random.uniform(start_x - 1.6, start_x + 1.6), 2)
                gy = round(random.uniform(start_y - 1.6, start_y + 1.6), 2)
                if math.hypot(gx - start_x, gy - start_y) < 0.6:
                    continue
                conflict = False
                for w in wall_configs:
                    if math.hypot(gx - w['pos'][0], gy - w['pos'][1]) < 0.45:
                        conflict = True
                        break
                for c in obstacle_coords:
                    if math.hypot(gx - c[0], gy - c[1]) < 0.45:
                        conflict = True
                        break
                for og in goals:
                    if math.hypot(gx - og[0], gy - og[1]) < 0.45:
                        conflict = True
                        break
                if not conflict:
                    goals.append([gx, gy])
            return goals

        goal_points = generate_random_goals(10)

        # 1. Aşama: Sahneye Görsel Öğeleri Ekle (İz Çizgisi, Hedef Küreleri, Engeller, Duvarlar)
        print("\n[Görselleştirme] Sahneye 10 Adet Dengeli Dağıtılmış Hedef Küresi, Silindirler ve DUVAR Engelleri ekleniyor...", flush=True)
        visualizer.setup_visuals(goal_points, [start_x, start_y])
        visualizer.add_obstacles(obstacle_coords)
        visualizer.add_walls(wall_configs)

        # 2. Aşama: GA ile En Kısa TSP Sırasını Çöz (10 Hedefli TSP)
        print("\n[GA] 10 Hedefli TSP Problemi için Genetik Algoritma çalıştırılıyor...", flush=True)
        ga = GeneticTSP(start_pos=[start_x, start_y], goal_points=goal_points, pop_size=60, generations=80)
        best_tour, est_distance = ga.solve()

        ordered_goals = [goal_points[i] for i in best_tour]
        ordered_goals.append([start_x, start_y])  # Başlangıç noktasına dönüş

        print(f"[GA] Optimum Rota Belirlendi: Ziyaret Sırası = {best_tour} -> Başlangıç", flush=True)
        print(f"[GA] Tahmini Rota Mesafesi: {est_distance:.2f} m\n", flush=True)

        # Metrik loglama için liste
        trajectory_log = []
        step_count = 0
        total_path_length = 0.0
        prev_x, prev_y = start_x, start_y
        goal_reach_threshold = 0.12  # metre (Robotun hedefin TAM MERKEZİNDEN geçmesini garanti eder)

        start_time = time.time()
        last_vl = 0.0
        last_vr = 0.0

        for goal_idx, target in enumerate(ordered_goals):
            target_x, target_y = target
            is_depot = (goal_idx == len(ordered_goals) - 1)
            target_name = "BAŞLANGIÇ (Depot)" if is_depot else f"Hedef #{goal_idx + 1} ({target_x:.2f}, {target_y:.2f})"
            print(f"\n>>> YENİ GÖREV: {target_name} yönüne ilerleniyor...", flush=True)

            goal_reached = False

            # Hedefe ulaşmak için yeterli adım sayısı
            for step in range(350):
                cur_x, cur_y, cur_yaw = robot.get_pose()

                # Sahneye robotun arkasından canlı kırmızı iz çiz
                visualizer.update_trail(cur_x, cur_y)

                # Yol uzunluğunu topla
                step_dist = math.hypot(cur_x - prev_x, cur_y - prev_y)
                total_path_length += step_dist
                prev_x, prev_y = cur_x, cur_y

                # Mesafe ve Açı Hatası
                dx = target_x - cur_x
                dy = target_y - cur_y
                pos_error = math.hypot(dx, dy)

                target_angle = math.atan2(dy, dx)
                heading_error = normalize_angle(target_angle - cur_yaw)

                # Ön sensörleri oku
                d_left, d_right = robot.read_front_sensors()

                # Hedefe son yaklaşmada (0.35m altı) robot doğrudan hedefe kilitlenir
                # Uzaktayken gerçek engeller için kaçış uygulanır
                is_obstacle = avoidance_ctrl.is_obstacle_detected(d_left, d_right) and (pos_error > 0.35)

                if is_obstacle:
                    v_obs_l, v_obs_r = avoidance_ctrl.evaluate(d_left, d_right)
                    v_tr_l, v_tr_r = tracking_ctrl.evaluate(pos_error, heading_error)
                    cmd_vl = 0.65 * v_obs_l + 0.35 * v_tr_l
                    cmd_vr = 0.65 * v_obs_r + 0.35 * v_tr_r
                    mode = "OBSTACLE_AVOIDANCE"
                else:
                    cmd_vl, cmd_vr = tracking_ctrl.evaluate(pos_error, heading_error)
                    mode = "TRACKING"

                # Hızları yumuşat (Low-pass filter ile ani titreme ve salınımları engelle)
                alpha = 0.45
                v_l = alpha * cmd_vl + (1.0 - alpha) * last_vl
                v_r = alpha * cmd_vr + (1.0 - alpha) * last_vr
                last_vl, last_vr = v_l, v_r

                # Hızları motora gönder
                robot.set_wheel_velocities(v_l, v_r)

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
                    print(f"[{mode:<18}] Kalan: {pos_error:.2f} m | Açı: {math.degrees(heading_error):+.1f}° | Sol: {d_left:.2f}m, Sağ: {d_right:.2f}m | L={v_l:.2f}, R={v_r:.2f}", flush=True)

                # Hedefe ulaşma kontrolü (Hedef küresinin tam göbeği)
                if pos_error < goal_reach_threshold:
                    goal_reached = True
                    print(f">>> TEBRİKLER: {target_name} noktasının TAM ÜSTÜNDEN GEÇİLDİ! (Hata: {pos_error:.3f} m)", flush=True)
                    break

                time.sleep(0.08)

            if not goal_reached:
                print(f">>> Zaman aşımı nedeniyle sonraki hedefe geçiliyor...", flush=True)

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
        robot.stop_motors()
        # Simülasyon bittiğinde yerdeki kırmızı çizgiyi, markerları ve engelleri tamamen temizle:
        visualizer.cleanup()
        robot.stop_simulation()
        print("Simülasyon tamamlandı ve yerdeki iz çizgisi temizlendi.", flush=True)


if __name__ == "__main__":
    run_navigation()
