from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")

print("CoppeliaSim bağlantısı başarılı!")
print("Simülasyon sürümü:", sim.getInt32Param(sim.intparam_program_version))