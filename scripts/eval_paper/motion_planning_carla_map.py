import logging
from pathlib import Path

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.solution import VehicleType
from commonroad.common.util import Interval
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Tag
from commonroad.scenario.state import InitialState, PMState
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from crcarla.carla_interface import CarlaInterface
from crcarla.controller.reactive_planner import ReactivePlannerInterface
from crcarla.helper.config import (
    CarlaParams,
    CustomVis,
    VehicleControlType,
    SupportedCARLAVersion,
    TrajectoryVisualization,
)

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

param = CarlaParams()
param.map = "Town06"
param.use_docker = False
param.carla_version = SupportedCARLAVersion.V_0_9_15
param.ego.vehicle_ks_state = False
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = False
param.vis_type = CustomVis.THIRD_PERSON
param.visualization.vis_activation.trajectory = TrajectoryVisualization.ALL
param.ego_view.record_video = True
param.ego_view.video_path = Path(__file__).parent.parent.parent
param.ego.carla_controller_type = VehicleControlType.TRANSFORM
param.simulation.max_time_step = 70
param.visualization.remove_tmp_files = False

rp_config = ReactivePlannerConfiguration()
rp_config.debug.draw_icons = True
rp_config.debug.save_plots = True
rp_config.debug.draw_traj_set = True  # needs to be set for CARLA visualization of trajectories
rp_config.debug.draw_ref_path = True
rp_config.debug.plots_file_format = "svg"
rp_config.planning.replanning_frequency = 1
rp_config.sampling.d_min = -1
rp_config.sampling.d_max = 1
rp_config.planning.time_steps_computation = 60
rp_config.general.path_output = str(Path(__file__).parent.parent)

ci = CarlaInterface(param)

# map can be created using the script create_cr_maps.py
scenario, _ = CommonRoadFileReader(Path(__file__).parent.parent.parent / "scenarios/ZAM_CARLATown06-1.xml").open()

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
    ReactivePlannerInterface(scenario, planning_problem, rp_config, draw_trajectories=True),
    None,
    None,
    planning_problem,
    VehicleType.BMW_320i,
)
sc = ci.create_cr_scenario()
CommonRoadFileWriter(
    sc,
    PlanningProblemSet([planning_problem]),
    author="Sebastian Maierhofer",
    affiliation="Technical University of Munich",
    source="CARLA",
    tags={Tag.URBAN},
).write_to_file(
    str(Path(__file__).parent.parent.parent / "scenarios/ZAM_CARLATown06-1-1_1_T-1.xml"), OverwriteExistingFile.ALWAYS
)
