import os
import logging

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import VehicleType

from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams, CustomVis
from carlacr.controller.reactive_planner import ReactivePlannerInterface
from commonroad_rp.utility.config import ReactivePlannerConfiguration


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


or_map = os.path.dirname(__file__) + "/../maps/DEU_Test-1_1_T-1.xodr"
cr_map = os.path.dirname(__file__) + "/../scenarios/DEU_Test-1_1_T-3.xml"
scenario, planning_problem_set = CommonRoadFileReader(cr_map).open()
param = CarlaParams()
param.map = or_map
param.ego.vehicle_ks_state = False
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = True
param.vis_type = CustomVis.EGO

rp_config = ReactivePlannerConfiguration()
rp_config.debug.draw_traj_set = True
rp_config.debug.save_plots = True
#rp_config.sampling.num_sampling_levels = 4
rp_config.sampling.v_max = 50
rp_config.sampling.v_min = 0
#rp_config.sampling.t_min = 0.2
rp_config.sampling.d_min = -5
rp_config.sampling.d_max = 5
rp_config.general.path_output = "/home/maierhofer/code/commonroad-carla-interface/"
ci = CarlaInterface(param)

planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
ci.plan(ReactivePlannerInterface(scenario, planning_problem, rp_config), None, scenario,
        list(planning_problem_set.planning_problem_dict.values())[0], VehicleType.BMW_320i)
