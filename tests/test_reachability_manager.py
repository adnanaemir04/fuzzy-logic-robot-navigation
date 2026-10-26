"""
test_reachability_manager.py
----------------------------
Reachability-Aware Task Manager birim ve durum makinesi testleri:
1. Hedef durum geçişleri (PENDING -> REACHABLE -> UNREACHABLE / COMPLETED)
2. Kapalı 4 duvarlı oda topolojik tespit testi
3. İlerleme takibi ve durgunluk (stagnation) tespiti
4. Raporlama ve istatistik doğrulaması
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
from reachability_manager import ReachabilityManager, GoalStatus


class TestReachabilityManager(unittest.TestCase):
    def setUp(self):
        # 4 duvarlı kapalı oda: Merkez (2.65, 3.10), yarıçap 0.45m
        self.enclosed_rooms = [{
            'center': (2.65, 3.10),
            'half_width': 0.45,
            'half_height': 0.45
        }]
        self.mgr = ReachabilityManager(
            stagnation_distance_threshold=0.10,
            stagnation_step_limit=5,
            max_attempts_per_goal=2,
            enclosed_rooms=self.enclosed_rooms
        )

    def test_topological_unreachable_detection(self):
        goals = [
            [0.5, 1.0],      # Ulaşılabilir
            [2.65, 3.10],    # Kapalı oda içinde
            [4.0, 1.5]       # Ulaşılabilir
        ]
        start_pos = [0.0, 0.0]
        self.mgr.register_goals(goals, start_pos)

        # Hedef 0 ve 2 ulaşılabilir olmalı
        self.assertEqual(self.mgr.goals[0].status, GoalStatus.REACHABLE)
        self.assertEqual(self.mgr.goals[2].status, GoalStatus.REACHABLE)

        # Hedef 1 kapalı odada olduğu için UNREACHABLE olmalı
        self.assertEqual(self.mgr.goals[1].status, GoalStatus.UNREACHABLE)
        self.assertIn("Topology", self.mgr.goals[1].failure_reason)

    def test_completion_status(self):
        goals = [[1.0, 1.0]]
        self.mgr.register_goals(goals, [0.0, 0.0])
        self.mgr.start_goal(0, (0.0, 0.0))
        self.mgr.mark_completed(0, 10.0)

        self.assertEqual(self.mgr.goals[0].status, GoalStatus.COMPLETED)

    def test_stagnation_and_perimeter_exploration(self):
        goals = [[2.65, 3.10]]
        self.mgr.register_goals(goals, [0.0, 0.0])
        self.mgr.start_goal(0, (2.65, 2.40))  # Odanın hemen dışında

        # 1. İlk yaklaşmada diğer cepheyi keşfetmek için alternatif nokta üretir:
        should_abort, reason, alt_wp = self.mgr.update_progress(
            robot_pos=(2.65, 2.40),
            d_left=0.35,
            d_right=0.35,
            current_time=5.0,
            sim_step_dist=0.01
        )
        self.assertFalse(should_abort)
        self.assertEqual(reason, "PERIMETER_EXPLORE_NEXT_SIDE")
        self.assertIsNotNone(alt_wp)

    def test_summary_report(self):
        goals = [[1.0, 1.0], [2.65, 3.10]]
        self.mgr.register_goals(goals, [0.0, 0.0])
        self.mgr.mark_completed(0, 5.0)

        report = self.mgr.get_summary_report()
        self.assertEqual(report["total_goals"], 2)
        self.assertEqual(report["completed_count"], 1)
        self.assertEqual(report["unreachable_count"], 1)
        self.assertEqual(report["completion_rate_pct"], 50.0)


if __name__ == "__main__":
    unittest.main()
