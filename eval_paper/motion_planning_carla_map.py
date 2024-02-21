import logging
from pathlib import Path

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import VehicleType
from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.state import InitialState, PMState
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from carlacr.carla_interface import CarlaInterface
from carlacr.controller.reactive_planner import ReactivePlannerInterface
from carlacr.helper.config import CarlaParams, CustomVis, VehicleControlType

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

param = CarlaParams()
param.map = "Town06"
param.ego.vehicle_ks_state = False
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = True
param.vis_type = CustomVis.EGO
param.ego_view.record_video = True
param.simulation.tm.osm_mode = True
param.ego_view.video_path = str(Path(__file__).parent.parent)
param.ego.carla_controller_type = VehicleControlType.TRANSFORM
param.simulation.max_time_step = 120

rp_config = ReactivePlannerConfiguration()
rp_config.debug.draw_icons = True
rp_config.debug.save_plots = True
rp_config.debug.draw_ref_path = True
rp_config.debug.plots_file_format = "svg"

rp_config.planning.replanning_frequency = 1
rp_config.sampling.d_min = -1
rp_config.sampling.d_max = 1
rp_config.planning.time_steps_computation = 60
rp_config.general.path_output = str(Path(__file__).parent.parent)
ci = CarlaInterface(param)

scenario, _ = CommonRoadFileReader(Path(__file__).parent.parent / "scenarios/ZAM_CARLATown06-1.xml").open()

planning_problem = PlanningProblem(
    planning_problem_id=0,
    initial_state=InitialState(
        time_step=0,
        position=np.array([275.0, -141.0]),
        orientation=0.0,
        velocity=8,
        acceleration=0.0,
        yaw_rate=0.0,
        slip_angle=0.0,
    ),
    goal_region=GoalRegion([PMState(time_step=Interval(0, 120), position=Rectangle(10, 3, np.array([400, -145.5])))]),
)
ci.plan(
    ReactivePlannerInterface(scenario, planning_problem, rp_config), None, None, planning_problem, VehicleType.BMW_320i
)
