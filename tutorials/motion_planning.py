from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import VehicleType
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from crcarla.carla_interface import CarlaInterface
from crcarla.controller.reactive_planner import ReactivePlannerInterface
from crcarla.helper.config import CarlaParams, CustomVis

# specify map an scenario
scenario, planning_problem_set = CommonRoadFileReader("scenarios/DEU_Test-1_1_T-2.xml").open()

# configure carla-interface
param = CarlaParams()
param.map = "maps/DEU_Test-1_1_T-2.xodr"
param.ego.vehicle_ks_state = False
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = True
param.vis_type = CustomVis.EGO
param.simulation.max_time_step = 300

# configure CommonRoad reactive planner
rp_config = ReactivePlannerConfiguration()
rp_config.debug.draw_traj_set = False
rp_config.debug.draw_icons = True
rp_config.debug.save_plots = True
rp_config.debug.plots_file_format = "svg"

# sampling params
rp_config.sampling.v_max = 10
rp_config.sampling.v_min = -10
rp_config.sampling.d_min = -3
rp_config.sampling.d_max = 3

# planning params
rp_config.planning.replanning_frequency = 1
rp_config.planning.time_steps_computation = 60
rp_config.sampling.num_sampling_levels = 4

# init carla interface
ci = CarlaInterface(param)

# get planning problem and remove ego vehicle from scenario
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
scenario.remove_obstacle(scenario.dynamic_obstacles[0])

# start planning
ci.plan(
    ReactivePlannerInterface(scenario, planning_problem, rp_config),
    None,
    scenario,
    list(planning_problem_set.planning_problem_dict.values())[0],
    VehicleType.BMW_320i,
)
