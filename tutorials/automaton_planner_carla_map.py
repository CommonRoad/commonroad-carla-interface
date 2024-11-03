from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import VehicleType

from crcarla.carla_interface import CarlaInterface
from crcarla.controller.automaton_planner.automatonPlannerInterface import AutomatonPlannerInterface, PlannerType
from crcarla.helper.config import CarlaParams, CustomVis, VehicleControlType

cr_scenario_path = Path(__file__).parents[1] / "scenarios/ZAM_Zip-1_19_T-1.xml"

scenario, pps = CommonRoadFileReader(cr_scenario_path).open()

# Configure simulation and scenario settings
param = CarlaParams()
param.map = cr_scenario_path
param.vis_type = CustomVis.THIRD_PERSON
param.ego.vehicle_ks_state = False
param.vehicle.vehicle_ks_state = False
param.offscreen_mode = True
param.vis_type = CustomVis.THIRD_PERSON
param.ego_view.record_video = True
param.ego_view.video_path = Path(__file__).parent.parent
param.ego.carla_controller_type = VehicleControlType.TRANSFORM
param.simulation.max_time_step = 200

# load parameter for the car
params = {'length': 4.3,
         'width': 1.7,
         'wheelbase': 2.3,
         'a_max': 9,  # maximum acceleration
         's_max': 1,  # maximum steering angle
         'svel_max': 0.4,
          'horizon': 3.0}
params['b'] = params['wheelbase']/2       # distance from car center to rear axis

# init carla interface
carla_interface = CarlaInterface(param)

# start planning
carla_interface.plan(
    AutomatonPlannerInterface(scenario, params, PlannerType.OPTIMIZATION),
    None,
    scenario,
    list(pps.planning_problem_dict.values())[0],
    VehicleType.BMW_320i,
)
