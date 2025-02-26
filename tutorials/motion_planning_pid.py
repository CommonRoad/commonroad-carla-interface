from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import VehicleType
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from crcarla.carla_interface import CarlaInterface
from crcarla.controller.reactive_planner import ReactivePlannerInterface
from crcarla.helper.config import CarlaParams, CustomVis, VehicleControlType

# specify map an scenario
scenario, planning_problem_set = CommonRoadFileReader("scenarios/DEU_Test-1_1_T-2.xml").open()

# configure carla-interface
param = CarlaParams()
param.map = "maps/DEU_Test-1_1_T-2.xodr"
param.ego.vehicle_ks_state = False
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = False
param.ego_view.record_video = False
param.ego.carla_controller_type = VehicleControlType.PID
param.vis_type = CustomVis.THIRD_PERSON
param.simulation.max_time_step = 300
param.simulation.time_step = 0.0
param.sync = False

param.ego.control.basic_control_pid_lat_kp = 0.3
param.ego.control.basic_control_pid_lat_ki = 0.0
param.ego.control.basic_control_pid_lat_kd = 0.0
param.ego.control.basic_control_pid_max_steering = 0.8

param.ego.control.basic_control_pid_lon_kp = 0.5
param.ego.control.basic_control_pid_lon_ki = 0.01
param.ego.control.basic_control_pid_lon_kd = 0.01
param.ego.control.basic_control_pid_max_break = 0.01

# configure CommonRoad reactive planner
rp_config = ReactivePlannerConfiguration()
rp_config.debug.draw_traj_set = True
rp_config.debug.draw_icons = True
rp_config.debug.save_plots = False
rp_config.debug.show_plots = False
rp_config.debug.show_evaluation_plots = False
rp_config.debug.multiproc = False

# sampling params
rp_config.sampling.v_max = 10
rp_config.sampling.v_min = -10
rp_config.sampling.d_min = -3
rp_config.sampling.d_max = 3

# planning params
rp_config.planning.replanning_frequency = 1
rp_config.planning.time_steps_computation = 60
rp_config.sampling.num_sampling_levels = 4
rp_config.planning.safety_margin_dynamic_obstacles = 0.5

# init carla interface
ci = CarlaInterface(param)

# get planning problem and remove ego vehicle from scenario
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
# scenario.remove_obstacle(scenario.dynamic_obstacles[0])

# start planning
ci.plan(
    ReactivePlannerInterface(scenario, planning_problem, rp_config),
    None,
    scenario,
    list(planning_problem_set.planning_problem_dict.values())[0],
    VehicleType.BMW_320i,
)
