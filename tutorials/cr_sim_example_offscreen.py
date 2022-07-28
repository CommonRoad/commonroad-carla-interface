import time
import carla
import subprocess
import logging
from carlacr.interface.carla_interface import CarlaInterface
import os
from carlacr.configurations.set_configs import set_configs

# Load config files
config = set_configs()
sleep_time = config.config_carla.sleep_time

# Run Carla Server with OFFScreen mode
with subprocess.Popen([config.config_carla.carla_path, '-RenderOffScreen']):
    time.sleep(sleep_time)

    logger = logging.getLogger(__name__)
    logging.basicConfig(level=logging.DEBUG)
    map_path = os.path.dirname(os.path.abspath(__file__)) + "/../maps/"
    scenario_path = os.path.dirname(os.path.abspath(__file__)) + "/../scenarios/"

    map_name = "four_way_crossing"
    scenario_name = "four_way_crossing_Modi"

    client = carla.Client(config.carla_config.host, config.carla_config.port)
    ci = CarlaInterface(map_path + map_name + ".xodr", client, scenario_path + scenario_name + ".xml", None)

    ci.load_map()

    time.sleep(sleep_time)  # time to move your view in carla-window

    ci.setup_carla(hybrid_physics_mode=False)

    StartTime = time.time()
    ci.run_scenario(clean_up=True, carla_vehicles=0, carla_pedestrians=0)
    ExecutionTime = (time.time() - StartTime)

    print('Execution time in seconds: ' + str(ExecutionTime))
