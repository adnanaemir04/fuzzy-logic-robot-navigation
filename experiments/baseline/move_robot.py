import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# CoppeliaSim'e bağlan
client = RemoteAPIClient()
sim = client.require('sim')

# Motorları bul (Doğru hiyerarşi yolları)
left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')

hiz = 2.0

# 1 SANİYE İLERİ
print("1 saniye ileri...")
sim.setJointTargetVelocity(left_motor, hiz)
sim.setJointTargetVelocity(right_motor, hiz)
time.sleep(1)

# 1 SANİYE DUR
print("1 saniye dur...")
sim.setJointTargetVelocity(left_motor, 0.0)
sim.setJointTargetVelocity(right_motor, 0.0)
time.sleep(1)

# 1 SANİYE GERİ
print("1 saniye geri...")
sim.setJointTargetVelocity(left_motor, -hiz)
sim.setJointTargetVelocity(right_motor, -hiz)
time.sleep(1)

# GÖREV BİTİŞİ (Tamamen durdur)
print("İşlem bitti, motorlar durduruldu.")
sim.setJointTargetVelocity(left_motor, 0.0)
sim.setJointTargetVelocity(right_motor, 0.0)