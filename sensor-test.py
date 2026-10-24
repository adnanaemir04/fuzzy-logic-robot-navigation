import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# CoppeliaSim'e bağlan
client = RemoteAPIClient()
sim = client.require('sim')

sensors = []

# Sensörleri hiyerarşideki köşeli parantezli isimleriyle bul
# CoppeliaSim'de numaralandırma genelde 0'dan başlar (0-15 arası)
for i in range(16):
    sensor_name = f'/PioneerP3DX/ultrasonicSensor[{i}]'
    try:
        sensor_handle = sim.getObject(sensor_name)
        sensors.append(sensor_handle)
    except Exception:
        # Eğer bazı numaralar yoksa atla
        continue

print(f"Toplam {len(sensors)} adet sensör başarıyla bulundu!")

# Sürekli okuma döngüsü
try:
    while True:
        # Ön taraftaki iki orta sensörü (örneğin indeks 3 ve 4) okuyalım
        for i in [3, 4]:
            sensor_handle = sensors[i]

            # Sensörü oku
            res, dist, point, obj, n = sim.readProximitySensor(sensor_handle)

            if res > 0:
                print(f"Sensör [{i}] engel algıladı! Mesafe: {dist:.2f} metre")
            else:
                print(f"Sensör [{i}] önü temiz.")

        print("-" * 30)
        time.sleep(0.5)  # Yarım saniyede bir güncelle

except KeyboardInterrupt:
    print("Okuma durduruldu.")