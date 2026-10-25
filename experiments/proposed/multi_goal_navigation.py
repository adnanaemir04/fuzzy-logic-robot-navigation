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

        # 1. Kullanıcının Çizdiği Kroki Ortamı ve Hedefleri:
        # Mavi Nokta: Robotun başlangıç ve bitiş üssü (start_x, start_y)
        visualizer.setup_sketch_environment(origin=[start_x, start_y])

        # Çizimdeki 11 Hedef Noktasının Krokiye Göre Tam Genişletilmiş Koordinatları:
        # Sol kanat (X ~ -0.4 -> 1.5):
        # Orta koridor (X ~ 1.8 -> 3.7):
        # Sağ kanat (X ~ 4.4 -> 5.0): En sağ noktalar iyice sağa kaydırıldı
        goal_points = [
            [start_x + 0.45, start_y + 0.55],   # G0: Sol alt hedef (Mavi noktanın hafif sağ-üstü)
            [start_x - 0.35, start_y + 1.25],   # G1: En sol orta hedef (Sol dış bükey kavis)
            [start_x + 0.25, start_y + 2.25],   # G2: Sol üst hedef (Sol kutunun solunda)
            [start_x + 1.35, start_y + 2.65],   # G3: Sol üst kutunun sağındaki tepe hedef
            [start_x + 1.85, start_y + 0.25],   # G4: Siyah çember hedef (Sol dikey duvarın hemen alt ucu)
            [start_x + 2.65, start_y + 1.80],   # G5: Orta koridor iç hedef
            [start_x + 2.90, start_y + 0.30],   # G6: Orta alt hedef (Kırmızı dikey duvarın sol altı)
            [start_x + 3.75, start_y + 2.55],   # G7: Sağ dikey duvarın üst başı hedef
            [start_x + 4.90, start_y + 2.50],   # G8: En sağ üst hedef (Yatay kutunun üstü)
            [start_x + 4.50, start_y + 1.40],   # G9: Sağ iç hedef (Yatay kutunun altı)
            [start_x + 5.00, start_y + 0.30],   # G10: En sağ alt köşe hedef
        ]

        print("\n[Görselleştirme] Kullanıcının Çizdiği 11 Hedef Küresi ve Kroki Ortamı Sahneye Ekleniyor...", flush=True)
        visualizer.setup_visuals(goal_points, [start_x, start_y])

        # 2. Aşama: GA (Genetik Algoritma) ile En Kısa TSP Sırasını Çöz
        print("\n[GA] 11 Hedefli Çizim Rotası için Genetik Algoritma çalıştırılıyor...", flush=True)
        ga = GeneticTSP(start_pos=[start_x, start_y], goal_points=goal_points, pop_size=60, generations=80)
        best_tour, est_distance = ga.solve()

        ordered_goals = [goal_points[i] for i in best_tour]
        ordered_goals.append([start_x, start_y])  # Başlangıç noktasına dönüş (Mavi nokta)

        print(f"[GA] Optimum Rota Belirlendi: Ziyaret Sırası = {best_tour} -> Mavi Nokta (Depot)", flush=True)
        print(f"[GA] Tahmini Rota Mesafesi: {est_distance:.2f} m\n", flush=True)

        trajectory_log = []
        step_count = 0
        total_path_length = 0.0
        prev_x, prev_y = start_x, start_y
        goal_reach_threshold = 0.22

        start_time = time.time()
        last_vl = 0.0
        last_vr = 0.0

        for goal_idx, target in enumerate(ordered_goals):
            target_x, target_y = target
            is_depot = (goal_idx == len(ordered_goals) - 1)
            target_name = "BAŞLANGIÇ (Depot)" if is_depot else f"Hedef #{goal_idx + 1} ({target_x:.2f}, {target_y:.2f})"
            print(f"\n>>> YENİ GÖREV: {target_name} yönüne ilerleniyor...", flush=True)

            goal_reached = False

            # 1. ÖNCE HEDEFE DÖNÜŞ (DÜZ GİTME GARANTİSİ):
            # Robot bir sonraki hedefe hareket etmeden önce yerinde durur ve tam hedefe döner.
            # Böylece kavisli gitmez, hedefe dön -> ok gibi düz git!
            robot.stop_motors()
            time.sleep(0.05)
            for align_step in range(80):
                cur_x, cur_y, cur_yaw = robot.get_pose()
                dx = target_x - cur_x
                dy = target_y - cur_y
                target_angle = math.atan2(dy, dx)
                head_err = normalize_angle(target_angle - cur_yaw)

                if abs(math.degrees(head_err)) < 4.0:
                    break

                # Yerinde dönüş hızı (açı yaklaştıkça yumuşat)
                turn_mag = 1.6 if abs(math.degrees(head_err)) > 20.0 else 0.9
                if head_err > 0:
                    robot.set_wheel_velocities(-turn_mag, turn_mag)
                else:
                    robot.set_wheel_velocities(turn_mag, -turn_mag)
                time.sleep(0.02)

            robot.stop_motors()
            time.sleep(0.04)
            last_vl, last_vr = 0.0, 0.0

            # 2. ŞİMDİ HEDEFE OK GİBİ DÜMDÜZ İLERLE:
            for step in range(350):
                cur_x, cur_y, cur_yaw = robot.get_pose()
                visualizer.update_trail(cur_x, cur_y)

                step_dist = math.hypot(cur_x - prev_x, cur_y - prev_y)
                total_path_length += step_dist
                prev_x, prev_y = cur_x, cur_y

                dx = target_x - cur_x
                dy = target_y - cur_y
                pos_error = math.hypot(dx, dy)

                # Hedefin TAM ÜSTÜNE BASILDIĞI AN BİTİR:
                if pos_error < goal_reach_threshold:
                    goal_reached = True
                    print(f">>> TEBRİKLER: {target_name} noktasının TAM ÜSTÜNDEN GEÇİLDİ! (Mesafe: {pos_error:.3f} m)", flush=True)
                    break

                target_angle = math.atan2(dy, dx)
                heading_error = normalize_angle(target_angle - cur_yaw)

                d_left, d_right = robot.read_front_sensors()

                # Engel kaçışı: Hedefe son 0.35m kala devre dışı
                is_obstacle = avoidance_ctrl.is_obstacle_detected(d_left, d_right) and (pos_error > 0.35)

                if is_obstacle:
                    cmd_vl, cmd_vr = avoidance_ctrl.evaluate(d_left, d_right)
                    mode = "OBSTACLE_AVOIDANCE"
                else:
                    # DÜZ GİT: Önceden hizalandığı için dümdüz ilerler
                    # Ufak sapmalarda oransal mikro-düzeltme ile düz çizgide tutar, geniş kavis yapmaz
                    base_v = 2.4
                    if pos_error < 0.35:
                        base_v = max(1.0, base_v * (pos_error / 0.35))

                    deg_err = math.degrees(heading_error)
                    if abs(deg_err) < 5.0:
                        cmd_vl, cmd_vr = base_v, base_v
                    else:
                        # Düz çizgiyi korumak için hafif oransal düzeltme (kavis çizmeden hedefe kilitli tutar)
                        kp = 0.035
                        diff = max(-0.8, min(0.8, kp * deg_err))
                        cmd_vl = base_v - diff
                        cmd_vr = base_v + diff
                    mode = "TRACKING"

                # Hızları yumuşat (Low-pass filter)
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
