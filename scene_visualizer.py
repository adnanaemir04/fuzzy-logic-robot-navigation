"""
scene_visualizer.py
-------------------
CoppeliaSim sahnesinde kullanıcı için zengin görselleştirme sağlar:
1. Robotun arkasında anlık kırmızı/mavi takip izi (Trajectory Trail Line) çizer.
2. Hedef noktalarına renkli küre bayrakları (Yeşil Goal Marker'lar) yerleştirir.
3. Sahneye makaledeki gibi engeller (Kırmızı silindirler / kutular) yerleştirir.
"""

class SceneVisualizer:
    def __init__(self, sim):
        self.sim = sim
        self.drawing_handle = None
        self.obstacle_handles = []
        self.goal_marker_handles = []
        self.prev_pos = None

    def setup_visuals(self, goal_points, start_pos):
        """Çizgi nesnesini ve hedef kürelerini oluşturur."""
        # Eski çizim nesnesi varsa temizle
        if self.drawing_handle is not None:
            try:
                self.sim.removeDrawingObject(self.drawing_handle)
            except Exception:
                pass

        # 1. Robotun gittiği yolu çizen Çizgi Nesnesi (Canlı Kırmızı-Sarı Kalın İz)
        # Format: lines, lineSize=5, color=[1.0, 0.2, 0.0] (parlak turuncu-kırmızı)
        self.drawing_handle = self.sim.addDrawingObject(
            self.sim.drawing_lines,
            5,      # Çizgi kalınlığı
            0.0,
            -1,     # Dünya koordinatları
            5000,   # Maksimum nokta kapasitesi
            [1.0, 0.1, 0.1]  # Kırmızı renk
        )

        # 2. Hedef Noktalarına Yeşil Parlak Küreler (Goal Markers) Ekle
        for idx, (gx, gy) in enumerate(goal_points):
            try:
                marker = self.sim.createPrimitiveShape(self.sim.primitiveshape_spheroid, [0.25, 0.25, 0.25])
                self.sim.setObjectPosition(marker, -1, [float(gx), float(gy), 0.15])
                self.sim.setShapeColor(marker, '', self.sim.colorcomponent_ambient_diffuse, [0.1, 0.9, 0.2])  # Yeşil
                self.sim.setObjectAlias(marker, f"GoalMarker_{idx+1}")
                self.goal_marker_handles.append(marker)
            except Exception:
                pass

        # Başlangıç noktası için Mavi Küre
        try:
            depot_marker = self.sim.createPrimitiveShape(self.sim.primitiveshape_spheroid, [0.25, 0.25, 0.25])
            self.sim.setObjectPosition(depot_marker, -1, [float(start_pos[0]), float(start_pos[1]), 0.15])
            self.sim.setShapeColor(depot_marker, '', self.sim.colorcomponent_ambient_diffuse, [0.2, 0.4, 1.0])  # Mavi
            self.sim.setObjectAlias(depot_marker, "DepotMarker")
            self.goal_marker_handles.append(depot_marker)
        except Exception:
            pass

        self.prev_pos = [start_pos[0], start_pos[1], 0.05]

    def add_obstacles(self, obstacle_coords):
        """
        Sahneye makaledeki gibi fiziksel ve görsel engeller ekler (Kırmızı silindirler).
        obstacle_coords: [[x1, y1], [x2, y2], ...]
        """
        for i, (ox, oy) in enumerate(obstacle_coords):
            try:
                obs = self.sim.createPrimitiveShape(self.sim.primitiveshape_cylinder, [0.35, 0.35, 0.6])
                self.sim.setObjectPosition(obs, -1, [float(ox), float(oy), 0.3])
                self.sim.setShapeColor(obs, '', self.sim.colorcomponent_ambient_diffuse, [0.9, 0.1, 0.1])  # Kırmızı engel
                self.sim.setObjectAlias(obs, f"VisualObstacle_{i+1}")
                self.obstacle_handles.append(obs)
            except Exception:
                pass

    def update_trail(self, cur_x, cur_y):
        """Robotun her adımında arkasına çizgi ekler."""
        if self.drawing_handle is None or self.prev_pos is None:
            self.prev_pos = [cur_x, cur_y, 0.05]
            return

        cur_pos = [cur_x, cur_y, 0.05]
        # Robot hareket ettiyse araya çizgi parçası ekle
        line_segment = [
            self.prev_pos[0], self.prev_pos[1], self.prev_pos[2],
            cur_pos[0], cur_pos[1], cur_pos[2]
        ]
        self.sim.addDrawingObjectItem(self.drawing_handle, line_segment)
        self.prev_pos = cur_pos

    def cleanup(self):
        """Simülasyon bitince geçici marker ve engelleri temizler."""
        for h in self.goal_marker_handles:
            try:
                self.sim.removeObject(h)
            except Exception:
                pass
        for h in self.obstacle_handles:
            try:
                self.sim.removeObject(h)
            except Exception:
                pass
        self.goal_marker_handles.clear()
        self.obstacle_handles.clear()
