import time
import carla
import subprocess
import logging
from carlacr.interface.carla_interface import CarlaInterface
from commonroad.common.file_reader import CommonRoadFileReader
from carlacr.helper.config import CarlaParams


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


or_map = "/media/sebastian/TUM/06_code/internal/carla-interface/maps/four_way_crossing.xodr"
cr_map = "/media/sebastian/TUM/06_code/internal/carla-interface/scenarios/four_way_crossing_Modi.xml"
# or_map = "/media/sebastian/TUM/06_code/internal/carla-interface/maps/DEU_Test-1_1_T-1.xodr"
# cr_map = "/media/sebastian/TUM/06_code/internal/carla-interface/scenarios/DEU_Test-1_1_T-1.xml"
# or_map = "Town"
cr_scenario, _ = scenario, planning_problem_set = CommonRoadFileReader(cr_map).open()
param = CarlaParams()
param.map = or_map
param.offscreen_mode = False

try:
    ci = CarlaInterface(param)
    ci.replay(cr_scenario)
except Exception as e:
    print(e)



#
# sleep_time = config.config_carla.sleep_time
#
# # Run Carla Server with 3D mode
# with subprocess.Popen([config.config_carla.carla_path]):
#     time.sleep(sleep_time)
#
#     logger = logging.getLogger(__name__)
#     logging.basicConfig(level=logging.DEBUG)
#
#     map_path = config.general.map_path
#     scenario_path = config.general.scenario_path
#
#     map_name = "four_way_crossing"
#     scenario_name = "four_way_crossing_Modi"
#
#     client =
#     ci = CarlaInterface(map_path + map_name + ".xodr", client, scenario_path + scenario_name + ".xml", None)
#
#     ci.load_map()
#
#     time.sleep(sleep_time)  # time to move your view in carla-window
#
#     ci.setup_carla(hybrid_physics_mode=False)
#
#     StartTime = time.time()
#     ci.run_scenario(clean_up=True, carla_vehicles=0, carla_pedestrians=0)
#     ExecutionTime = (time.time() - StartTime)
#
#     print('Execution time in seconds: ' + str(ExecutionTime))
