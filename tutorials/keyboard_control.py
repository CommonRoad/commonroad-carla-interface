import logging

from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams, CustomVis

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


# Specify CommonRoad and OpenDRIVE map
#or_map_path = os.path.dirname(__file__) + "/../maps/DEU_Test-1_1_T-1.xodr"
#cr_scenario_path = os.path.dirname(__file__) + "/../scenarios/DEU_Test-1_1_T-1.xml"

# Configure simulation and scenario settings
#scenario, planning_problem_set = CommonRoadFileReader(cr_scenario_path).open()
param = CarlaParams()
#param.map = or_map_path
#param.map = "Town10HD"
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = True # set to false if your system is powerful enough
param.vis_type = CustomVis.EGO # set to false if your system is powerful enough
param.simulation.record_video = False
param.simulation.max_time_step = 1200

# Initialize CARLA-Interface and start keyboard control
ci = CarlaInterface(param)
ci.keyboard_control() #scenario, list(planning_problem_set.planning_problem_dict.values())[0])

# Extract solution driven by vehicle and store it
# logger.info("Store solution")
# solution = ci.solution(list(planning_problem_set.planning_problem_dict.keys())[0], VehicleModel.PM,
#                        VehicleType.BMW_320i, CostFunction.JB1)
# CommonRoadSolutionWriter(Solution(scenario.scenario_id, [solution])).write_to_file(overwrite=True)
