import os
import logging
from carlacr.interface.carla_interface import CarlaInterface
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from carlacr.helper.config import CarlaParams

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# or_map = os.path.dirname(__file__) + "/../maps/four_way_crossing.xodr"
# cr_map = os.path.dirname(__file__) + "/../scenarios/four_way_crossing_Modi.xml"
or_map = os.path.dirname(__file__) + "/../maps/DEU_Test-1_1_T-1.xodr"
cr_map = os.path.dirname(__file__) + "/../scenarios/DEU_Test-1_1_T-1.xml"
or_map = "Town01"
scenario, planning_problem_set = CommonRoadFileReader(cr_map).open()
param = CarlaParams()
param.map = or_map
param.obstacle.vehicle_ks_state = False
param.offscreen_mode = True
param.birds_eye_view = True


ci = CarlaInterface(param)
# ci.keyboard_control(scenario, list(planning_problem_set.planning_problem_dict.values())[0])
# ci.keyboard_control()
sc, pps = ci.scenario_generation(ci.create_cr_map())
CommonRoadFileWriter(sc, pps).write_to_file()


