import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def disable_child_script_if_active(sim, robot_handle):
    """
    CoppeliaSim PioneerP3DX modelinin üzerindeki varsayılan Braitenberg Lua scriptini
    devre dışı bırakır. Bu script temizlenmezse, her simülasyon adımında motor hızını
    v0=2.0 olarak ezerek robotun durmasını engeller.
    """
    try:
        script = sim.getScript(sim.scripttype_childscript, robot_handle)
        text = sim.getScriptStringParam(script, sim.scriptstringparam_text)
        # Eğer varsayılan Braitenberg motor kontrolü varsa temizle
        if "braitenberg" in text.lower() or "v0=2" in text or "v0 = 2" in text:
            print("PioneerP3DX içindeki varsayılan otonom Lua scripti tespit edildi. Python kontrolü için temizleniyor...")
            empty_script = (
                "sim = require('sim')\n"
                "function sysCall_init()\nend\n"
                "function sysCall_actuation()\nend\n"
                "function sysCall_sensing()\nend\n"
                "function sysCall_cleanup()\nend\n"
            )
            sim.setScriptStringParam(script, sim.scriptstringparam_text, empty_script)
            print("Gömülü Lua scripti başarıyla nötralize edildi.")
    except Exception as e:
        # Script zaten yoksa veya erişilemiyorsa yola devam et
        pass

def main():
    client = RemoteAPIClient()
    sim = client.require("sim")

    # Robot ve motor nesnelerini bul
    robot = sim.getObject("/PioneerP3DX")
    left_motor = sim.getObject("/PioneerP3DX/leftMotor")
    right_motor = sim.getObject("/PioneerP3DX/rightMotor")

    # Robotun gömülü Lua scripti Python komutlarını ezmesin diye kontrol et
    disable_child_script_if_active(sim, robot)

    sim.startSimulation()
    time.sleep(1)

    try:
        # 1. İleri hareket komutu (1.0 m/s rad/s)
        print("İleri komutu gönderildi (hız = 1.0)...")
        sim.setJointTargetVelocity(left_motor, 1.0)
        sim.setJointTargetVelocity(right_motor, 1.0)
        time.sleep(2)

        # 2. Sıfır hız komutu (Durdurma)
        print("\nSıfır hız komutu gönderildi (hız = 0.0)...")
        sim.setJointTargetVelocity(left_motor, 0.0)
        sim.setJointTargetVelocity(right_motor, 0.0)

        target_l = sim.getJointTargetVelocity(left_motor)
        target_r = sim.getJointTargetVelocity(right_motor)
        print(f"Sol motor hedef hızı: {target_l:.3f}")
        print(f"Sağ motor hedef hızı: {target_r:.3f}")

        # 3. Gerçek motor hızlarının 0'a ulaştığını adım adım doğrula
        print("\nGerçek motor hızları takip ediliyor:")
        stopped_successfully = False
        for i in range(10):
            left_velocity = sim.getJointVelocity(left_motor)
            right_velocity = sim.getJointVelocity(right_motor)

            print(f"[{i+1}/10] Sol motor anlık hız: {left_velocity:+.3f} rad/s | Sağ motor anlık hız: {right_velocity:+.3f} rad/s")
            
            # Ataletten sonra hız sıfıra indiyse
            if abs(left_velocity) < 0.01 and abs(right_velocity) < 0.01:
                stopped_successfully = True
            
            time.sleep(0.5)

        if stopped_successfully:
            print("\n>>> BAŞARILI: Robotun her iki motoru da tamamen durduruldu (Hız: 0.000 rad/s).")
        else:
            print("\n>>> UYARI: Robot beklenen sürede durmadı.")

    finally:
        # Simülasyonu sonlandır
        sim.stopSimulation()
        print("Simülasyon durduruldu. Test tamamlandı.")

if __name__ == "__main__":
    main()
