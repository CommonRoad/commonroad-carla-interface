import os
import logging

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import VehicleType

from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams, CustomVis
from carlacr.controller.reactive_planner import ReactivePlannerInterface


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


# or_map = os.path.dirname(__file__) + "/../maps/four_way_crossing.xodr"
# cr_map = os.path.dirname(__file__) + "/../scenarios/four_way_crossing_Modi.xml"
or_map = os.path.dirname(__file__) + "/../maps/DEU_Test-1_1_T-1.xodr"
cr_map = os.path.dirname(__file__) + "/../scenarios/DEU_Test-1_1_T-1.xml"
# or_map = "Town01"
scenario, planning_problem_set = CommonRoadFileReader(cr_map).open()
param = CarlaParams()
param.map = or_map
param.ego.vehicle_ks_state = False
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = True
param.vis_type = CustomVis.BIRD


ci = CarlaInterface(param)
ci.plan(ReactivePlannerInterface(), scenario,
        list(planning_problem_set.planning_problem_dict.values())[0], VehicleType.BMW_320i)
