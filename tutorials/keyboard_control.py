import os
import logging

from carlacr.interface.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams, CustomVis

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import Solution, VehicleType, VehicleModel, CostFunction
from commonroad.common.solution import CommonRoadSolutionWriter

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


# Specify CommonRoad and OpenDRIVE map
or_map_path = os.path.dirname(__file__) + "/../maps/DEU_Test-1_1_T-1.xodr"
cr_scenario_path = os.path.dirname(__file__) + "/../scenarios/DEU_Test-1_1_T-1.xml"

# Configure simulation and scenario settings
scenario, planning_problem_set = CommonRoadFileReader(cr_scenario_path).open()
param = CarlaParams()
param.map = or_map_path
param.obstacle.vehicle_ks_state = False
param.offscreen_mode = True # set to false if your system is powerful enough
param.vis_type = CustomVis.BIRD # set to false if your system is powerful enough

# Initialize CARLA-Interface and start keyboard control
ci = CarlaInterface(param)
ci.keyboard_control(scenario, list(planning_problem_set.planning_problem_dict.values())[0])

# Extract solution driven by vehicle and store it
logger.info("Store solution")
solution = ci.solution(list(planning_problem_set.planning_problem_dict.keys())[0], VehicleModel.PM,
                       VehicleType.BMW_320i, CostFunction.JB1)
CommonRoadSolutionWriter(Solution(scenario.scenario_id, [solution])).write_to_file(overwrite=True)
