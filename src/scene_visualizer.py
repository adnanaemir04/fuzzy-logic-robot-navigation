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

    def _clear_all_drawing_objects(self):
        """Sahnedeki tüm çizim nesnelerini (kırmızı iz çizgilerini) siler."""
        if self.drawing_handle is not None:
            try:
                self.sim.removeDrawingObject(self.drawing_handle)
            except Exception:
                pass
            self.drawing_handle = None
        # Olası eski oturumlardan kalan tüm çizim handles havuzunu temizle
        for h in range(50):
            try:
                self.sim.removeDrawingObject(h)
            except Exception:
                pass

    def setup_visuals(self, goal_points, start_pos):
        """Çizgi nesnesini ve hedef kürelerini oluşturur."""
        # 0. Sahnedeki eski çizgi kalıntılarını tamamen temizle
        self._clear_all_drawing_objects()

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
                # Küreler sadece görseldir; sensörler ve fiziksel çarpışma bunu engel olarak görmemeli:
                self.sim.setObjectSpecialProperty(marker, self.sim.objectspecialproperty_renderable)
                self.goal_marker_handles.append(marker)
            except Exception:
                pass

        # Başlangıç noktası için Mavi Küre
        try:
            depot_marker = self.sim.createPrimitiveShape(self.sim.primitiveshape_spheroid, [0.25, 0.25, 0.25])
            self.sim.setObjectPosition(depot_marker, -1, [float(start_pos[0]), float(start_pos[1]), 0.15])
            self.sim.setShapeColor(depot_marker, '', self.sim.colorcomponent_ambient_diffuse, [0.2, 0.4, 1.0])  # Mavi
            self.sim.setObjectAlias(depot_marker, "DepotMarker")
            self.sim.setObjectSpecialProperty(depot_marker, self.sim.objectspecialproperty_renderable)
            self.goal_marker_handles.append(depot_marker)
        except Exception:
            pass

        self.prev_pos = [start_pos[0], start_pos[1], 0.05]

    def add_obstacles(self, obstacle_coords):
        """
        Sahneye silindir engeller ekler (Kırmızı silindirler).
        obstacle_coords: [[x1, y1], [x2, y2], ...]
        """
        for i, (ox, oy) in enumerate(obstacle_coords):
            try:
                obs = self.sim.createPrimitiveShape(self.sim.primitiveshape_cylinder, [0.4, 0.4, 0.6])
                self.sim.setObjectPosition(obs, -1, [float(ox), float(oy), 0.3])
                self.sim.setShapeColor(obs, '', self.sim.colorcomponent_ambient_diffuse, [0.9, 0.1, 0.1])  # Kırmızı silindir
                self.sim.setObjectAlias(obs, f"VisualObstacle_{i+1}")
                # Engeller sensör tarafından algılanabilir (detectable) ve fiziksel çarpışabilir (collidable) olmalıdır:
                props = (self.sim.objectspecialproperty_collidable | 
                         self.sim.objectspecialproperty_measurable | 
                         self.sim.objectspecialproperty_detectable | 
                         self.sim.objectspecialproperty_renderable)
                self.sim.setObjectSpecialProperty(obs, props)
                # Fiziksel respondable yap (Robot içinden geçemez):
                self.sim.setObjectInt32Param(obs, self.sim.shapeintparam_respondable, 1)
                self.obstacle_handles.append(obs)
            except Exception:
                pass

    def add_walls(self, wall_configs):
        """
        Sahneye dikdörtgen prizma / duvar engelleri ekler.
        wall_configs: [{'pos': [x, y], 'size': [lx, ly, lz]}, ...]
        """
        for i, w in enumerate(wall_configs):
            try:
                wx, wy = w['pos']
                sx, sy, sz = w.get('size', [1.5, 0.25, 0.6])
                wall = self.sim.createPrimitiveShape(self.sim.primitiveshape_cuboid, [float(sx), float(sy), float(sz)])
                self.sim.setObjectPosition(wall, -1, [float(wx), float(wy), sz / 2.0])
                # Duvar rengi
                color = w.get('color', [0.85, 0.4, 0.1])
                self.sim.setShapeColor(wall, '', self.sim.colorcomponent_ambient_diffuse, color)
                self.sim.setObjectAlias(wall, f"WallObstacle_{i+1}")
                # Duvarlar sensör tarafından algılanabilir ve fiziksel çarpışabilir olmalıdır:
                props = (self.sim.objectspecialproperty_collidable | 
                         self.sim.objectspecialproperty_measurable | 
                         self.sim.objectspecialproperty_detectable | 
                         self.sim.objectspecialproperty_renderable)
                self.sim.setObjectSpecialProperty(wall, props)
                # Fiziksel respondable yap (Robot duvarın içinden geçemez):
                self.sim.setObjectInt32Param(wall, self.sim.shapeintparam_respondable, 1)
                self.obstacle_handles.append(wall)
            except Exception:
                pass

    def setup_sketch_environment(self, origin=[0.0, 0.0]):
        """
        Kullanıcının çizdiği krokinin birebir ortamı:
        - Mavi nokta sol alt köşe (origin: [ox, oy])
        - 1. Siyah Dikey Duvar (Sol kanat ile orta arasında)
        - 2. Kırmızı Dikey Duvar (Orta ile sağ kanat arasında)
        - 3. Kırmızı Dikdörtgen Kutu Engeller (Sol üst, Üst orta, Sağ kanat)
        """
        ox, oy = origin
        
        # 1. Duvarlar (Siyah Sol Dikey Duvar & Kırmızı Sağ Dikey Duvar)
        walls = [
            # Sol Dikey Duvar (Siyah Çizgi - Çizimde aşağısı açık, Y=0.55'ten Y=2.35'e uzanır, boy: 1.8m):
            {"pos": [ox + 1.60, oy + 1.45], "size": [0.08, 1.8, 0.6], "color": [0.1, 0.1, 0.1]},
            # Sağ Dikey Duvar (Kırmızı Çizgi - Çizimde aşağısı açık, Y=0.55'ten Y=2.55'e uzanır, boy: 2.0m):
            {"pos": [ox + 3.80, oy + 1.55], "size": [0.08, 2.0, 0.6], "color": [0.9, 0.1, 0.1]},
        ]
        self.add_walls(walls)

        # 2. Kırmızı Dikdörtgen Kutu Engeller (Kullanıcı çizimi):
        red_boxes = [
            # Sol üstteki kare/dikdörtgen kırmızı kutu (G1 ve G2 arasında, sol duvarın solunda):
            {"pos": [ox + 1.15, oy + 2.70], "size": [0.45, 0.40, 0.5]},
            # Üst ortadaki dikey kırmızı kutu (G2 ve G4 arasında - üstten kestirme dönüş için ferah koridor):
            {"pos": [ox + 2.65, oy + 3.25], "size": [0.35, 0.50, 0.5]},
            # Sağdaki yatay kırmızı kutu (Sağ duvarın sağında, G8 ile G9 arasında yatay dikdörtgen):
            {"pos": [ox + 4.90, oy + 2.05], "size": [0.75, 0.35, 0.5]},
        ]
        for i, box in enumerate(red_boxes):
            try:
                bx, by = box['pos']
                sx, sy, sz = box['size']
                b = self.sim.createPrimitiveShape(self.sim.primitiveshape_cuboid, [float(sx), float(sy), float(sz)])
                self.sim.setObjectPosition(b, -1, [float(bx), float(by), sz / 2.0])
                self.sim.setShapeColor(b, '', self.sim.colorcomponent_ambient_diffuse, [0.95, 0.1, 0.1])
                self.sim.setObjectAlias(b, f"SketchRedBox_{i+1}")
                self.obstacle_handles.append(b)
            except Exception:
                pass
            except Exception:
                pass

    def update_trail(self, cur_x, cur_y):
        """Robotun her adımında arkasına canlı kırmızı çizgi ekler."""
        # Çizginin zeminin veya robotun altında kalmaması için Z = 0.14m (zemin yüzeyinin üstü)
        z_height = 0.14
        if self.drawing_handle is None or self.prev_pos is None:
            self.prev_pos = [cur_x, cur_y, z_height]
            return

        cur_pos = [cur_x, cur_y, z_height]
        # Robot hareket ettiyse araya çizgi parçası ekle
        line_segment = [
            self.prev_pos[0], self.prev_pos[1], self.prev_pos[2],
            cur_pos[0], cur_pos[1], cur_pos[2]
        ]
        self.sim.addDrawingObjectItem(self.drawing_handle, line_segment)
        self.prev_pos = cur_pos

    def ensure_large_floor(self, target_size=14.0):
        """
        CoppeliaSim zeminini (Floor ve altındaki dama tahtası 'box' nesnesi)
        robotun ve tüm hedeflerin rahatça sığacağı boyuta görsel ve fiziksel olarak genişletir.
        """
        for name in ['/Floor', './Floor', 'Floor', '/box', './box', 'box']:
            try:
                floor = self.sim.getObject(name)
                bb = self.sim.getShapeBB(floor)[0]
                current_size = max(bb[0], bb[1])
                if current_size < target_size:
                    scale = target_size / current_size
                    self.sim.scaleObject(floor, scale, scale, 1.0)
                    print(f"[Zemin] Zemin ({name}) {current_size:.1f}m -> {target_size:.1f}m genişletildi.", flush=True)
            except Exception:
                pass

    def cleanup(self):
        """Simülasyon bitince kırmızı çizgi izini, hedef kürelerini ve geçici engelleri temizler."""
        # 1. Kırmızı yol izini tamamen sahneden kaldır
        self._clear_all_drawing_objects()

        # 2. Hedef kürelerini kaldır
        for h in self.goal_marker_handles:
            try:
                self.sim.removeObject(h)
            except Exception:
                pass
        self.goal_marker_handles.clear()

        # 3. Geçici engelleri kaldır
        for h in self.obstacle_handles:
            try:
                self.sim.removeObject(h)
            except Exception:
                pass
        self.obstacle_handles.clear()
        self.prev_pos = None
