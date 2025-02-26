from commonroad.common.file_reader import CommonRoadFileReader

from crcarla.carla_interface import CarlaInterface
from crcarla.helper.config import CarlaParams, CustomVis

# Specify CommonRoad scenario
scenario, pps = CommonRoadFileReader(cr_scenario_path := "scenarios/DEU_Test-1_1_T-2.xml").open()

# Configure simulation and scenario settings
param = CarlaParams()
param.map = cr_scenario_path
param.vis_type = CustomVis.THIRD_PERSON

# Execute simulation for replaying CommonRoad scenario
CarlaInterface(param).replay(scenario, ego_id=6)
