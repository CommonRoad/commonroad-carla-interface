from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import VehicleType

from commonroad_idm_planner.configuration.planner_config import IDMConfigFactory

from crcarla.carla_interface import CarlaInterface
from crcarla.controller.idm_planner import IDMPlannerInterface
from crcarla.helper.config import CarlaParams, CustomVis

# specify map an scenario
scenario, planning_problem_set = CommonRoadFileReader("scenarios/DEU_Test-1_1_T-2.xml").open()

# configure carla-interface
param = CarlaParams()
param.map = "maps/DEU_Test-1_1_T-2.xodr"
param.ego.vehicle_ks_state = False
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = True
param.vis_type = CustomVis.THIRD_PERSON
param.simulation.max_time_step = 300

config = IDMConfigFactory().generate_default_config()

# init carla interface
ci = CarlaInterface(param)

# get planning problem and remove ego vehicle from scenario
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
scenario.remove_obstacle(scenario.dynamic_obstacles[0])

# start planning
ci.plan(
    IDMPlannerInterface(scenario, planning_problem, config),
    None,
    scenario,
    list(planning_problem_set.planning_problem_dict.values())[0],
    VehicleType.BMW_320i,
)
