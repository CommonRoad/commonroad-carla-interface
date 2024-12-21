from crcarla.carla_interface import CarlaInterface
from crcarla.helper.config import CarlaParams, CustomVis, EgoPlanner

# Example for using steering wheel in CARLA Town10

# set parameters
param = CarlaParams()
param.vis_type = CustomVis.THIRD_PERSON

# Initialize CARLA-Interface and start keyboard control
ci = CarlaInterface(param)
ci.external_control(EgoPlanner.STEERING_WHEEL)
