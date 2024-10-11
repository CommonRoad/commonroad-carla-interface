from pathlib import Path

import numpy as np
from commonroad.common.solution import VehicleType
from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.state import InitialState, PMState
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from crcarla.carla_interface import CarlaInterface
from crcarla.controller.reactive_planner import ReactivePlannerInterface
from crcarla.helper.config import CarlaParams, CustomVis, VehicleControlType

# Carla params
param = CarlaParams()
param.map = "Town01"
param.ego.vehicle_ks_state = False
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = True
param.vis_type = CustomVis.EGO
param.ego_view.record_video = True
param.ego_view.video_path = Path(__file__).parent.parent
param.ego.carla_controller_type = VehicleControlType.TRANSFORM
param.simulation.max_time_step = 200

# configure CommonRoad reactive planner
# debug params
rp_config = ReactivePlannerConfiguration()
rp_config.debug.draw_icons = True
rp_config.debug.save_plots = True
rp_config.debug.draw_ref_path = True
rp_config.debug.show_plots = True
rp_config.debug.draw_traj_set = True
rp_config.debug.num_workers = 6
rp_config.debug.plots_file_format = "svg"
rp_config.general.path_output = str(Path(__file__).parent.parent)

# planning params
rp_config.planning.dt = 0.1
rp_config.planning.replanning_frequency = 2

# vehicle
# rp_config.vehicle.id_type_vehicle = 2

# sampling
rp_config.sampling.sampling_method = 1
rp_config.sampling.t_min = 0.4
rp_config.sampling.v_max = 30
rp_config.sampling.d_min = -1
rp_config.sampling.d_max = 1

# init carla interface
carla_interface = CarlaInterface(param)

# get scenario from specified map
scenario = carla_interface.create_cr_map()

# set planning problem for previously specified map
planning_problem = PlanningProblem(
    planning_problem_id=0,
    initial_state=InitialState(
        time_step=0,
        position=np.array([275.0, -134.0]),
        orientation=0.0,
        velocity=8,
        acceleration=0.0,
        yaw_rate=0.0,
        slip_angle=0.0,
    ),
    goal_region=GoalRegion(
        [PMState(time_step=Interval(0, 250), position=Rectangle(10, 3, np.array([335, -165.0]), -1.57))]
    ),
)

# start planning
carla_interface.plan(
    ReactivePlannerInterface(scenario, planning_problem, rp_config),
    None,
    scenario,
    planning_problem,
    VehicleType.BMW_320i,
)
