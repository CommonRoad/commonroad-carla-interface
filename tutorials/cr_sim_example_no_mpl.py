import time
import os
import sys
import logging
import carla
from carlacr.interface.carla_interface import CarlaInterface
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath("__file__"))))

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)
map_path = os.path.dirname(os.path.abspath(__file__)) + "/maps/"
scenario_path = os.path.dirname(os.path.abspath(__file__)) + "/scenarios/"

name = "DEU_Test-1_1_T-1"

client = carla.Client('localhost', 2000)
ci = CarlaInterface(map_path + name + ".xodr",
                    client, scenario_path + name + ".xml", None)

ci.load_map()

time.sleep(5)  # time to move your view in carla-window

ci.setup_carla(hybrid_physics_mode=False)

StartTime = time.time()
ci.run_scenario(clean_up=True, carla_vehicles=0, carla_pedestrians=0)
ExecutionTime = (time.time() - StartTime)

print('Execution time in seconds: ' + str(ExecutionTime))
