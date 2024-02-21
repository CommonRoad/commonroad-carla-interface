from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams, CustomVis, EgoPlanner

# Example for using steering wheel in CARLA Town10

# set parameters
param = CarlaParams()
param.vis_type = CustomVis.EGO

# Initialize CARLA-Interface and start keyboard control
ci = CarlaInterface(param)
ci.external_control(EgoPlanner.STEERING_WHEEL)
