import time
import carla
from carlacr.interface.carla_interface import CarlaInterface
import logging
logger=logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)
map_path = "../maps/"
scenario_path = "../scenarios/"

name = "DEU_Test-1_1_T-1"

client = carla.Client('localhost', 2000)
ci = CarlaInterface(scenario_path + name + ".xodr", map_path + name + ".xml", client, None)

ci.load_map()

time.sleep(5)  # time to move your view in carla-window

ci.setup_carla(hybrid_physics_mode=False)

startTime = time.time()
ci.run_scenario(clean_up=True, carla_vehicles=0, carla_pedestrians=0)
executionTime = (time.time() - startTime)

print('Execution time in seconds: ' + str(executionTime))
