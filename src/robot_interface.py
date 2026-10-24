"""
robot_interface.py
------------------
Pioneer P3-DX robotu için CoppeliaSim Remote API arayüzü:
- Konum ve oryantasyon okuma (x, y, yaw)
- Sol ve Sağ motor hızlarını güvenli şekilde gönderme
- Ön ultrasonik sensörleri okuma (Sol küme ve Sağ küme mesafe ayrıştırması)
- Gömülü Lua betiğini otomatik nötralize etme
"""

import math
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class PioneerRobot:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        # Robot ve bileşen handles
        self.robot = self.sim.getObject("/PioneerP3DX")
        self.left_motor = self.sim.getObject("/PioneerP3DX/leftMotor")
        self.right_motor = self.sim.getObject("/PioneerP3DX/rightMotor")

        # Makalede kullanılan ön 6 ultrasonik sensör
        # Indeksler: 1, 2, 3 (Sol ön), 4, 5, 6 (Sağ ön)
        self.sensors = []
        for i in range(16):
            try:
                handle = self.sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]")
                self.sensors.append(handle)
            except Exception:
                pass

        # Robot üzerindeki varsayılan Braitenberg Lua scriptini temizle
        self._disable_child_script()

    def _disable_child_script(self):
        try:
            script = self.sim.getScript(self.sim.scripttype_childscript, self.robot)
            text = self.sim.getScriptStringParam(script, self.sim.scriptstringparam_text)
            if "braitenberg" in text.lower() or "v0=2" in text or "v0 = 2" in text:
                empty_script = (
                    "sim = require('sim')\n"
                    "function sysCall_init()\nend\n"
                    "function sysCall_actuation()\nend\n"
                    "function sysCall_sensing()\nend\n"
                    "function sysCall_cleanup()\nend\n"
                )
                self.sim.setScriptStringParam(script, self.sim.scriptstringparam_text, empty_script)
        except Exception:
            pass

    def start_simulation(self):
        self.sim.startSimulation()
        time.sleep(0.5)

    def stop_simulation(self):
        self.stop_motors()
        self.sim.stopSimulation()

    def get_pose(self):
        """
        Robotun anlık (x, y, yaw) koordinatlarını döndürür.
        yaw: [-pi, pi] radyan aralığında
        """
        pos = self.sim.getObjectPosition(self.robot, -1)
        ori = self.sim.getObjectOrientation(self.robot, -1)
        x = pos[0]
        y = pos[1]
        yaw = ori[2]
        return x, y, yaw

    def set_wheel_velocities(self, v_left, v_right):
        """Motorlara hedef hız (rad/s) komutunu gönderir."""
        self.sim.setJointTargetVelocity(self.left_motor, float(v_left))
        self.sim.setJointTargetVelocity(self.right_motor, float(v_right))

    def stop_motors(self):
        """Motorları sıfırlar ve durdurur."""
        self.set_wheel_velocities(0.0, 0.0)

    def read_front_sensors(self):
        """
        Makaledeki gibi ön sensörleri Sol ve Sağ grup olarak okur.
        Sol grup: sensör 1, 2, 3
        Sağ grup: sensör 4, 5, 6
        Dönen: (d_left_min, d_right_min) metre cinsinden
        """
        max_range = 2.0
        left_readings = []
        right_readings = []

        # Sol ön sektör (sensör 1, 2, 3)
        for idx in [1, 2, 3]:
            if idx < len(self.sensors):
                res, dist, _, _, _ = self.sim.readProximitySensor(self.sensors[idx])
                if res > 0 and dist > 0:
                    left_readings.append(dist)
                else:
                    left_readings.append(max_range)

        # Sağ ön sektör (sensör 4, 5, 6)
        for idx in [4, 5, 6]:
            if idx < len(self.sensors):
                res, dist, _, _, _ = self.sim.readProximitySensor(self.sensors[idx])
                if res > 0 and dist > 0:
                    right_readings.append(dist)
                else:
                    right_readings.append(max_range)

        d_left = min(left_readings) if left_readings else max_range
        d_right = min(right_readings) if right_readings else max_range

        return d_left, d_right
