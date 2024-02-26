from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams, CustomVis

# Specify CommonRoad scenario
cr_scenario_path = str(Path(__file__).parent.parent / "scenarios/DEU_Test-1_1_T-2.xml")
scenario, _ = CommonRoadFileReader(cr_scenario_path).open()

# Configure simulation and scenario settings
param = CarlaParams()
param.map = cr_scenario_path
param.vis_type = CustomVis.EGO

# Execute simulation for replaying CommonRoad scenario
CarlaInterface(param).replay(scenario, ego_id=6)
