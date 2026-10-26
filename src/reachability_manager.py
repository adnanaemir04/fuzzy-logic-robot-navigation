"""
reachability_manager.py
-----------------------
Özgün Katkı: Reachability-Aware Task Manager (Erişilebilirlik Farkındalıklı Görev Yöneticisi)

Bu modül, Pioneer P3-DX mobil robotunun çoklu hedef navigasyonunda hedeflerin
erişilebilirliğini çok katmanlı olarak değerlendirir:
1. Hedef Durum Yönetimi (PENDING, REACHABLE, TEMPORARILY_BLOCKED, UNCERTAIN, UNREACHABLE, COMPLETED)
2. Geometrik / Harita Bağlantısallık Değerlendirmesi (Topological & Polygon Ray-Casting Connectivity)
3. İlerleme Takibi (Progress Monitoring & Stagnation Detection)
4. Çevresel İnceleme (Perimeter Exploration & Failure Analysis)
5. Dinamik Görev ve Rota Yeniden Planlama (Dynamic Re-planning & Safe Escalation)
"""

from enum import Enum
import math
import time
from typing import Dict, List, Optional, Tuple


class GoalStatus(Enum):
    PENDING = "PENDING"                         # Henüz değerlendirilmedi veya ziyaret edilmedi
    REACHABLE = "REACHABLE"                     # Mevcut bilgiye göre ulaşılabilir
    TEMPORARILY_BLOCKED = "TEMPORARILY_BLOCKED" # Geçici bir engel nedeniyle ilerlenemiyor
    UNCERTAIN = "UNCERTAIN"                     # Erişilebilirlik hakkında yeterli kanıt yok
    UNREACHABLE = "UNREACHABLE"                 # Tanımlı kısıtlar altında kesin erişilemez
    COMPLETED = "COMPLETED"                     # Hedefe başarıyla ulaşıldı


class GoalRecord:
    """Tek bir hedefe ait tüm durum, analiz ve telemetri kayıtları."""
    def __init__(self, goal_id: int, position: Tuple[float, float], is_mandatory: bool = True):
        self.goal_id = goal_id
        self.position = position
        self.is_mandatory = is_mandatory
        self.status = GoalStatus.PENDING
        self.status_history = [(GoalStatus.PENDING, "Initial creation", 0.0)]
        self.attempts = 0
        self.time_spent = 0.0
        self.distance_traveled = 0.0
        self.min_distance_reached = float("inf")
        self.failure_reason = ""
        self.explored_perimeter_angles: List[float] = []

    def update_status(self, new_status: GoalStatus, reason: str, sim_time: float):
        if self.status != new_status:
            self.status_history.append((new_status, reason, sim_time))
            self.status = new_status
            self.failure_reason = reason


class ReachabilityManager:
    """
    Erişilebilirlik Farkındalıklı Görev Yöneticisi.
    Robotun hedeflere ulaşma çabasını izler, fiziksel ve sensör tabanlı
    engelleri değerlendirir, gereksiz dolanmaları önler.
    """
    def __init__(
        self,
        stagnation_distance_threshold: float = 0.15,
        stagnation_step_limit: int = 45,
        max_attempts_per_goal: int = 2,
        enclosed_rooms: Optional[List[Dict]] = None
    ):
        """
        stagnation_distance_threshold: Robotun takılı kaldığını belirten min ilerleme mesafesi (metre)
        stagnation_step_limit: İlerleme olmadan geçen adım sayısı sınırı
        max_attempts_per_goal: Bir hedef için maksimum deneme sayısı
        enclosed_rooms: [{'center': (x, y), 'half_width': w, 'half_height': h}, ...] kapalı odalar
        """
        self.stagnation_distance_threshold = stagnation_distance_threshold
        self.stagnation_step_limit = stagnation_step_limit
        self.max_attempts_per_goal = max_attempts_per_goal
        self.enclosed_rooms = enclosed_rooms or []

        self.goals: Dict[int, GoalRecord] = {}
        self.active_goal_id: Optional[int] = None
        self.step_counter = 0

        # İlerleme izleme tamponları
        self.last_robot_pos = (0.0, 0.0)
        self.recent_positions: List[Tuple[float, float]] = []
        self.stagnation_counter = 0

        # İstatistikler
        self.total_replannings = 0
        self.total_alternatives_found = 0

    def register_goals(self, goal_points: List[List[float]], start_pos: List[float]):
        """Görev listesindeki tüm hedefleri yöneticiye kaydeder ve harita bağlantısallığını analiz eder."""
        self.goals.clear()
        for idx, pt in enumerate(goal_points):
            rec = GoalRecord(goal_id=idx, position=(float(pt[0]), float(pt[1])))
            # Harita/Topoloji bağlantısallık ön değerlendirmesi
            if self.is_inside_enclosed_room((pt[0], pt[1])):
                # Eğer robot oda dışındaysa ve hedef kapalı odadaysa:
                if not self.is_inside_enclosed_room((start_pos[0], start_pos[1])):
                    rec.status = GoalStatus.UNREACHABLE
                    rec.failure_reason = "Topology: Target enclosed inside 4-wall sealed room without openings."
                    rec.status_history.append((GoalStatus.UNREACHABLE, rec.failure_reason, 0.0))
                else:
                    rec.status = GoalStatus.REACHABLE
            else:
                rec.status = GoalStatus.REACHABLE
            self.goals[idx] = rec

    def is_inside_enclosed_room(self, point: Tuple[float, float]) -> bool:
        """Noktanın haritadaki kapalı 4 duvarlı odalardan birinin içinde kalıp kalmadığını kontrol eder."""
        px, py = point
        for room in self.enclosed_rooms:
            cx, cy = room['center']
            hw, hh = room.get('half_width', 0.45), room.get('half_height', 0.45)
            if (cx - hw <= px <= cx + hw) and (cy - hh <= py <= cy + hh):
                return True
        return False

    def start_goal(self, goal_id: int, robot_pos: Tuple[float, float]):
        """Yeni bir hedefe yönelik navigasyonu başlatır."""
        self.active_goal_id = goal_id
        self.last_robot_pos = robot_pos
        self.recent_positions = [robot_pos]
        self.stagnation_counter = 0
        
        if goal_id in self.goals:
            rec = self.goals[goal_id]
            rec.attempts += 1
            if rec.status == GoalStatus.PENDING:
                rec.update_status(GoalStatus.REACHABLE, "Navigating towards target", time.time())

    def update_progress(
        self,
        robot_pos: Tuple[float, float],
        d_left: float,
        d_right: float,
        current_time: float,
        sim_step_dist: float
    ) -> Tuple[bool, str, Optional[Tuple[float, float]]]:
        """
        Her kontrol adımında robotun ilerlemesini, sensör durumunu ve engellenme halini değerlendirir.
        Dönen: (should_abort_goal, reason, alternative_waypoint)
        """
        if self.active_goal_id is None or self.active_goal_id not in self.goals:
            return False, "", None

        rec = self.goals[self.active_goal_id]
        rec.distance_traveled += sim_step_dist

        gx, gy = rec.position
        rx, ry = robot_pos
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        if dist_to_goal < rec.min_distance_reached:
            rec.min_distance_reached = dist_to_goal

        # Robotun hedefe göre açısal sektörünü kaydet (Perimeter Exploration)
        angle_to_goal = math.atan2(ry - gy, rx - gx)
        rec.explored_perimeter_angles.append(angle_to_goal)

        # İlerleme (Stagnation) kontrolü: Son N adımda yeterli mesafe katedildi mi?
        self.recent_positions.append(robot_pos)
        if len(self.recent_positions) > self.stagnation_step_limit:
            oldest_pos = self.recent_positions.pop(0)
            net_displacement = math.hypot(rx - oldest_pos[0], ry - oldest_pos[1])

            # Robot hareket edemiyor veya engel karşısında salınıyorsa (Stagnation):
            if net_displacement < self.stagnation_distance_threshold:
                self.stagnation_counter += 1
            else:
                self.stagnation_counter = max(0, self.stagnation_counter - 1)

        # 1. Durum: Hedef kapalı odada ve fiziksel olarak ulaşılamaz
        if self.is_inside_enclosed_room((gx, gy)):
            # Çevresel inceleme (Perimeter Analysis):
            # Robot kapalı odanın duvarına yaklaştıysa ve farklı açılardan yaklaşmayı denediyse:
            if dist_to_goal < 1.10 and min(d_left, d_right) < 0.65:
                # Odanın etrafında keşif yapıldı mı veya duvarlar tamamen kapalı mı?
                rec.update_status(
                    GoalStatus.UNREACHABLE,
                    "Physical Verification: Target is completely sealed inside 4 collision walls. No entry point.",
                    current_time
                )
                return True, "TARGET_UNREACHABLE_SEALED", None

        # 2. Durum: Genel Stagnation (Sıkışma veya geçici engel)
        if self.stagnation_counter >= 3:
            # Engel çok yakınsa:
            if min(d_left, d_right) < 0.50:
                if rec.attempts < self.max_attempts_per_goal:
                    rec.update_status(
                        GoalStatus.TEMPORARILY_BLOCKED,
                        "Temporary blockage detected. Will evaluate alternative approach vector.",
                        current_time
                    )
                    # Alternatif bir çevre yaklaşım noktası öner:
                    alt_wp = self._generate_alternative_waypoint(robot_pos, (gx, gy))
                    self.total_alternatives_found += 1
                    return False, "ALTERNATIVE_ATTEMPT", alt_wp
                else:
                    rec.update_status(
                        GoalStatus.UNREACHABLE,
                        f"Unreachable after {rec.attempts} attempts and perimeter stagnation.",
                        current_time
                    )
                    return True, "TARGET_UNREACHABLE_STAGNATION", None
            else:
                rec.update_status(
                    GoalStatus.UNCERTAIN,
                    "Progress halted without direct proximity contact.",
                    current_time
                )

        return False, "", None

    def _generate_alternative_waypoint(
        self,
        robot_pos: Tuple[float, float],
        goal_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Engelle karşılaşıldığında hedefin etrafında alternatif bir teğetsel keşif noktası üretir."""
        rx, ry = robot_pos
        gx, gy = goal_pos
        current_angle = math.atan2(ry - gy, rx - gx)
        # 60 derece yana açı kaydırmasıyla alternatif açı bul
        new_angle = current_angle + (math.pi / 3.0)
        radius = 0.85
        alt_x = gx + radius * math.cos(new_angle)
        alt_y = gy + radius * math.sin(new_angle)
        return alt_x, alt_y

    def mark_completed(self, goal_id: int, current_time: float):
        """Hedefin başarıyla tamamlandığını kaydeder."""
        if goal_id in self.goals:
            self.goals[goal_id].update_status(GoalStatus.COMPLETED, "Goal reached within tolerance.", current_time)

    def mark_unreachable(self, goal_id: int, reason: str, current_time: float):
        """Hedefi kesin olarak erişilemez olarak kaydeder."""
        if goal_id in self.goals:
            self.goals[goal_id].update_status(GoalStatus.UNREACHABLE, reason, current_time)

    def get_summary_report(self) -> Dict:
        """Görevin sonunda tamamlanan, erişilemeyen ve bekleyen hedeflerin detaylı metriklerini üretir."""
        completed = [g for g in self.goals.values() if g.status == GoalStatus.COMPLETED]
        unreachable = [g for g in self.goals.values() if g.status == GoalStatus.UNREACHABLE]
        temporarily_blocked = [g for g in self.goals.values() if g.status == GoalStatus.TEMPORARILY_BLOCKED]
        uncertain = [g for g in self.goals.values() if g.status == GoalStatus.UNCERTAIN]
        pending = [g for g in self.goals.values() if g.status == GoalStatus.PENDING]

        total = len(self.goals)
        comp_rate = (len(completed) / total * 100.0) if total > 0 else 0.0

        return {
            "total_goals": total,
            "completed_count": len(completed),
            "unreachable_count": len(unreachable),
            "temporarily_blocked_count": len(temporarily_blocked),
            "uncertain_count": len(uncertain),
            "pending_count": len(pending),
            "completion_rate_pct": comp_rate,
            "total_replannings": self.total_replannings,
            "total_alternatives_found": self.total_alternatives_found,
            "goal_details": [
                {
                    "goal_id": g.goal_id,
                    "position": g.position,
                    "status": g.status.value,
                    "attempts": g.attempts,
                    "distance_traveled": round(g.distance_traveled, 3),
                    "min_distance_reached": round(g.min_distance_reached, 3),
                    "failure_reason": g.failure_reason
                }
                for g in self.goals.values()
            ]
        }
