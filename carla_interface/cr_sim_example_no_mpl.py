import time
import carla
from carla_interface.CarlaInterface import CarlaInterface

map_path = "../maps/"
scenario_path = "../scenarios/"

name = "DEU_Test-1_1_T-1"

client = carla.Client('localhost', 2000)
ci = CarlaInterface(scenario_path + name +".xml", map_path + name +".xodr", client, None)

ci.load_map()

time.sleep(5)  # time to move your view in carla-window

ci.setup_carla(hybrid_physics_mode=False)

startTime = time.time()
ci.run_scenario(clean_up=True, carla_vehicles=4, carla_pedestrians=0)
executionTime = (time.time() - startTime)

print('Execution time in seconds: ' + str(executionTime))