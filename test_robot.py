from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")

# Sahnedeki robotu bul
robot = sim.getObject("/PioneerP3DX")

print("Robot bulundu!")
print("Robot handle:", robot)