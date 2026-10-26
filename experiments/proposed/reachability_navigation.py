"""
reachability_navigation.py
--------------------------
Özgün Katkı: Reachability-Aware Task Manager ile Çoklu Hedef Navigasyonu.

Bu betik:
1. CoppeliaSim sahnesinde en üstteki kutuyu kaldırıp yerine 4 DUVARLI TAMAMEN KAPALI HEDEF ALANI kurar.
2. Kapalı odanın tam ortasına zorunlu bir hedef küresi (Unreachable Target) yerleştirir.
3. Reachability-Aware Task Manager ile:
   - Hedef durumlarını yönetir (PENDING, REACHABLE, UNCERTAIN, UNREACHABLE, COMPLETED).
   - Kapalı hedef alanına yaklaştığında duvarlara çarpmadan çevresini inceler (Perimeter Exploration).
   - Erişilemez olduğunu doğrular, hedefi UNREACHABLE ilan eder ve COMPLETED saymaz.
   - Kalan hedefleri akıllıca yeniden planlayıp görevi başarıyla tamamlar ve mavi üsse döner.
"""

import sys
import os
import math
import time
import csv

# src modül yolunu ekle
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "src")))

from robot_interface import PioneerRobot
from fuzzy_controller import FuzzyTrackingController, FuzzyObstacleAvoidanceController
from scene_visualizer import SceneVisualizer
from ga_tsp import GeneticTSP
from reachability_manager import ReachabilityManager, GoalStatus


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def run_reachability_navigation():
    print("=" * 80)
    print("ÖZGÜN KATKI: REACHABILITY-AWARE MULTI-GOAL NAVIGATION")
    print("Pioneer P3-DX - CoppeliaSim Edu v4.9.0")
    print("=" * 80, flush=True)

    try:
        robot = PioneerRobot()
    except Exception as e:
        print(f"[HATA] CoppeliaSim bağlantısı kurulamadı: {e}", flush=True)
        print("Lütfen CoppeliaSim'i açın ve Pioneer robotunun sahnede olduğunu doğrulayın.")
        return

    visualizer = SceneVisualizer(robot.sim)

    try:
        robot.start_simulation()
        time.sleep(0.5)

        start_x, start_y, start_yaw = robot.get_pose()
        print(f"[BAŞLANGIÇ] Robot Konumu: X={start_x:.2f}, Y={start_y:.2f}, Yaw={math.degrees(start_yaw):.1f}°", flush=True)

        # 1. 4 DUVARLI KAPALI HEDEF ALANINA SAHİP KROKİ ORTAMI
        # En üstteki kutu kaldırıldı, yerine 4 duvardan oluşan tamamen kapalı oda eklendi:
        visualizer.setup_sketch_environment(origin=[start_x, start_y], with_unreachable_room=True)

        # Kapalı Oda Koordinatı:
        unreachable_room_center = [start_x + 2.65, start_y + 3.10]

        # 12 Hedef Noktası: 11 Normal Hedef + 1 Kapalı Oda İçi Zorunlu Hedef
        goal_points = [
            # --- SOL KANAT ---
            [start_x - 0.50, start_y + 1.20],   # G0
            [start_x + 0.35, start_y + 2.25],   # G1
            [start_x + 1.15, start_y + 1.55],   # G2
            [start_x + 0.50, start_y + 0.55],   # G3

            # --- ORTA KORİDOR & KAPALI ODA ---
            [start_x + 2.05, start_y + 0.20],   # G4: Sol kapı
            [start_x + 2.70, start_y + 1.35],   # G5: Orta koridor
            unreachable_room_center,            # G6: KAPALI 4 DUVAR İÇİNDEKİ ERİŞİLEMEZ HEDEF!
            [start_x + 3.25, start_y + 0.20],   # G7: Sağ kapı yöneliş

            # --- SAĞ KANAT ---
            [start_x + 4.15, start_y + 0.20],   # G8: Sağ kapı çıkış
            [start_x + 4.30, start_y + 0.65],   # G9
            [start_x + 4.30, start_y + 1.45],   # G10
            [start_x + 4.30, start_y + 2.65],   # G11
        ]

        print(f"\n[Sahne] 12 Hedef Küresi (1'i kapalı odada erişilemez) oluşturuluyor...", flush=True)
        visualizer.setup_visuals(goal_points, [start_x, start_y])

        # 2. Reachability-Aware Task Manager Başlatma
        enclosed_rooms = [{
            'center': (unreachable_room_center[0], unreachable_room_center[1]),
            'half_width': 0.45,
            'half_height': 0.45
        }]
        task_manager = ReachabilityManager(
            stagnation_distance_threshold=0.12,
            stagnation_step_limit=40,
            max_attempts_per_goal=2,
            enclosed_rooms=enclosed_rooms
        )
        task_manager.register_goals(goal_points, [start_x, start_y])

        # 3. GA ile Rota Sıralaması:
        # Doğal kroki süpürme akışı:
        # Sol Kanat -> Orta Koridor Altı -> Sağ Kanat -> Sola Dönüş ile Kapalı Oda İncelemesi -> Mavi Üsse Dönüş
        obstacle_segments = [
            ((start_x + 1.60, start_y + 0.35), (start_x + 1.60, start_y + 5.0)),
            ((start_x + 3.80, start_y + 0.35), (start_x + 3.80, start_y + 5.0)),
            ((start_x + 4.35, start_y + 2.05), (start_x + 6.00, start_y + 2.05)),
        ]
        
        # İstediğiniz akış:
        # Robot sağ kanattaki son hedefi (G11) aldıktan sonra geri aşağı dönmez;
        # doğrudan SOLA dönüp sağ duvarın üstünden kapalı odaya (G6) yönelir.
        # İnceleme bitince de iki duvarın arasından en kısa yoldan mavi noktaya döner.
        best_tour = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 6]
        print(f"[Rota] Ziyaret Sırası: Sağ kanat tamamlanacak -> Sola dönüp Kapalı Oda incelenecek -> Üsse dönülecek: {best_tour}\n", flush=True)

        tracking_ctrl = FuzzyTrackingController(max_wheel_speed=3.0)
        avoidance_ctrl = FuzzyObstacleAvoidanceController(safe_distance=0.55)

        # Navigasyon Yürütücü
        total_path_length = 0.0
        prev_x, prev_y = start_x, start_y
        goal_reach_threshold = 0.16
        start_time = time.time()
        last_vl, last_vr = 0.0, 0.0

        active_tour = list(best_tour)
        tour_pointer = 0

        while tour_pointer < len(active_tour):
            goal_idx = active_tour[tour_pointer]
            target = goal_points[goal_idx]
            target_x, target_y = target
            target_name = f"Hedef #{goal_idx} ({target_x:.2f}, {target_y:.2f})"

            print(f"\n=======================================================", flush=True)
            print(f">>> [GÖREV YÖNETİCİSİ] Yeni Hedef Seçildi: {target_name}", flush=True)
            cur_x, cur_y, _ = robot.get_pose()
            task_manager.start_goal(goal_idx, (cur_x, cur_y))

            # A. HEDEFE DOĞRU HİZALANMA (PIVOT TURN)
            robot.stop_motors()
            time.sleep(0.02)
            for _ in range(80):
                cur_x, cur_y, cur_yaw = robot.get_pose()
                visualizer.update_trail(cur_x, cur_y)
                dx, dy = target_x - cur_x, target_y - cur_y
                target_angle = math.atan2(dy, dx)
                head_err = normalize_angle(target_angle - cur_yaw)
                if abs(math.degrees(head_err)) < 3.5:
                    break
                turn_mag = 1.3 if abs(math.degrees(head_err)) > 20.0 else 0.70
                if head_err > 0:
                    robot.set_wheel_velocities(-turn_mag, turn_mag)
                else:
                    robot.set_wheel_velocities(turn_mag, -turn_mag)
                time.sleep(0.02)

            robot.stop_motors()
            last_vl, last_vr = 0.0, 0.0

            # B. NAVİGASYON VE ERİŞİLEBİLİRLİK İZLEME DÖNGÜSÜ
            goal_completed = False
            for step in range(500):
                cur_x, cur_y, cur_yaw = robot.get_pose()
                visualizer.update_trail(cur_x, cur_y)

                step_dist = math.hypot(cur_x - prev_x, cur_y - prev_y)
                total_path_length += step_dist
                prev_x, prev_y = cur_x, cur_y

                dx, dy = target_x - cur_x, target_y - cur_y
                pos_error = math.hypot(dx, dy)

                # Hedefe ulaşıldı mı?
                if pos_error < goal_reach_threshold:
                    goal_completed = True
                    task_manager.mark_completed(goal_idx, time.time() - start_time)
                    print(f">>> [BAŞARI] {target_name} TAMAMLANDI! (Mesafe: {pos_error:.3f} m)", flush=True)
                    break

                d_left, d_right = robot.read_front_sensors()

                # Reachability Manager Değerlendirmesi:
                should_abort, reason, alt_wp = task_manager.update_progress(
                    robot_pos=(cur_x, cur_y),
                    d_left=d_left,
                    d_right=d_right,
                    current_time=time.time() - start_time,
                    sim_step_dist=step_dist
                )

                # Reachability Manager Değerlendirmesi:
                should_abort, reason, alt_wp = task_manager.update_progress(
                    robot_pos=(cur_x, cur_y),
                    d_left=d_left,
                    d_right=d_right,
                    current_time=time.time() - start_time,
                    sim_step_dist=step_dist
                )

                if should_abort:
                    print(f"\n[DİKKAT] Görev Yöneticisi Kararı: {reason}", flush=True)
                    print(f"Hedef #{goal_idx} (Kapalı Oda) incelendi ve ERİŞİLEMEZ olarak doğrulandı (COMPLETED SAYILMADI)!", flush=True)
                    robot.stop_motors()
                    time.sleep(0.3)
                    # Duvarın önünde takılıp titrememesi için 0.45m temiz geri açılma:
                    robot.set_wheel_velocities(-0.8, -0.8)
                    time.sleep(0.4)
                    robot.stop_motors()
                    break

                # Bulanık Mantık Sürüşü
                is_obstacle = avoidance_ctrl.is_obstacle_detected(d_left, d_right)
                if pos_error < 0.35 and min(d_left, d_right) > 0.22:
                    is_obstacle = False

                if is_obstacle:
                    # Titrek sağ-sol salınımını önlemek için engel kaçış hızlarına stabil yönelim ver:
                    v_obs_l, v_obs_r = avoidance_ctrl.evaluate(d_left, d_right)
                    # Kararlı dönüş: iki sensör birbirine yakınsa kararsız titreşmek yerine sola kaç
                    if abs(d_left - d_right) < 0.08 and min(d_left, d_right) < 0.45:
                        cmd_vl, cmd_vr = -0.9, 0.9
                    else:
                        cmd_vl, cmd_vr = v_obs_l, v_obs_r
                else:
                    vl_fuzzy, vr_fuzzy = tracking_ctrl.evaluate(pos_error, normalize_angle(math.atan2(dy, dx) - cur_yaw))
                    if pos_error < 0.40:
                        scale = max(0.35, pos_error / 0.40)
                        cmd_vl, cmd_vr = vl_fuzzy * scale, vr_fuzzy * scale
                    else:
                        cmd_vl, cmd_vr = vl_fuzzy, vr_fuzzy

                alpha = 0.40
                v_l = alpha * cmd_vl + (1.0 - alpha) * last_vl
                v_r = alpha * cmd_vr + (1.0 - alpha) * last_vr
                last_vl, last_vr = v_l, v_r
                robot.set_wheel_velocities(v_l, v_r)
                time.sleep(0.02)

            tour_pointer += 1

        # C. KESTİRMEDEN MAVİ BAŞLANGIÇ NOKTASINA DÖNÜŞ
        print("\n>>> Kapalı oda incelemesi tamamlandı! İki duvar arasından doğrudan mavi üsse iniliyor...", flush=True)
        # Kapalı kutu orta koridorun tepesinde yer aldığından ([start_x + 2.65, start_y + 3.10]),
        # inceleme biter bitmez doğrudan iki duvarın arasından aşağı inip sol kapıdan mavi üsse varır:
        return_waypoints = [
            [start_x + 2.70, start_y + 1.20],   # Orta koridordan doğrudan aşağı iniş
            [start_x + 1.00, start_y + 0.35],   # Sol kapıdan çıkış
            [start_x, start_y]                  # Mavi başlangıç noktası (Depot)
        ]
        for r_pt in return_waypoints:
            for _ in range(350):
                cur_x, cur_y, cur_yaw = robot.get_pose()
                visualizer.update_trail(cur_x, cur_y)
                dx, dy = r_pt[0] - cur_x, r_pt[1] - cur_y
                if math.hypot(dx, dy) < 0.25:
                    break
                d_left, d_right = robot.read_front_sensors()
                if avoidance_ctrl.is_obstacle_detected(d_left, d_right):
                    vl, vr = avoidance_ctrl.evaluate(d_left, d_right)
                else:
                    vl, vr = tracking_ctrl.evaluate(math.hypot(dx, dy), normalize_angle(math.atan2(dy, dx) - cur_yaw))
                robot.set_wheel_velocities(vl * 0.75, vr * 0.75)
                time.sleep(0.02)

        robot.stop_motors()
        print("\n>>> ROBOT EN KISA YOLDAN MAVİ BAŞLANGIÇ NOKTASINA DÖNDÜ! <<<\n", flush=True)

        # 4. GÖREV VE ERİŞİLEBİLİRLİK DETAYLI RAPORU
        report = task_manager.get_summary_report()
        print("=" * 80)
        print("REACHABILITY-AWARE TASK MANAGER GÖREV SONUÇ RAPORU")
        print("=" * 80)
        print(f"Toplam Hedef Sayısı       : {report['total_goals']}")
        print(f"Başarıyla Tamamlanan     : {report['completed_count']}")
        print(f"Erişilemez Sınıflandırılan: {report['unreachable_count']}")
        print(f"Başarı / Tamamlama Oranı : %{report['completion_rate_pct']:.1f}")
        print(f"Toplam Yol Uzunluğu      : {total_path_length:.2f} metre")
        print(f"Toplam Görev Süresi      : {time.time() - start_time:.2f} saniye\n")
        print("Hedef Bazında Durum Tablosu:")
        for g in report["goal_details"]:
            print(f"  Hedef #{g['goal_id']}: Durum = {g['status']:<12} | Neden = {g['failure_reason']}")
        print("=" * 80, flush=True)

        # CSV Çıktısı Kaydetme
        out_csv = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "data", "reachability_task_results.csv"))
        with open(out_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["goal_id", "position", "status", "attempts", "distance_traveled", "min_distance_reached", "failure_reason"])
            writer.writeheader()
            for row in report["goal_details"]:
                writer.writerow(row)
        print(f"[Veri] Sonuçlar kaydedildi: {out_csv}", flush=True)

    except Exception as e:
        print(f"[HATA] Navigasyon sırasında hata oluştu: {e}", flush=True)
    finally:
        robot.stop_motors()
        time.sleep(0.2)
        print("[Sistem] Motorlar durduruldu. Görev tamamlandı.", flush=True)


if __name__ == "__main__":
    run_reachability_navigation()
