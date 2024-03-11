import logging
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import VehicleType
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from crpred.basic_models.constant_velocity_predictor import (
    ConstantVelocityLinearPredictor,
)

from crcarla.carla_interface import CarlaInterface
from crcarla.controller.reactive_planner import ReactivePlannerInterface
from crcarla.helper.config import CarlaParams, CustomVis, VehicleControlType

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

predictor = ConstantVelocityLinearPredictor()


or_map = str(Path(__file__).parent.parent / "maps/DEU_Test-1_1_T-4.xodr")
cr_map = str(Path(__file__).parent.parent / "scenarios/DEU_Test-1_1_T-4.xml")
scenario, planning_problem_set = CommonRoadFileReader(cr_map).open()
param = CarlaParams()
param.map = or_map
param.ego.vehicle_ks_state = False
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = True
param.vis_type = CustomVis.EGO
param.ego_view.record_video = True
param.ego_view.video_path = str(Path(__file__).parent.parent)
param.ego.carla_controller_type = VehicleControlType.TRANSFORM
param.simulation.max_time_step = 120

rp_config = ReactivePlannerConfiguration()
rp_config.debug.draw_traj_set = False
rp_config.debug.draw_icons = True
rp_config.debug.save_plots = True
rp_config.debug.plots_file_format = "svg"

rp_config.planning.replanning_frequency = 1
rp_config.sampling.d_min = -3
rp_config.sampling.d_max = 3
rp_config.planning.time_steps_computation = 60
rp_config.sampling.num_sampling_levels = 4
rp_config.general.path_output = str(Path(__file__).parent.parent)
ci = CarlaInterface(param)

planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

ci.plan(
    ReactivePlannerInterface(scenario, planning_problem, rp_config),
    None,
    scenario,
    list(planning_problem_set.planning_problem_dict.values())[0],
    VehicleType.BMW_320i,
)
