from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams, CustomVis

# Specify CommonRoad and OpenDRIVE map
cr_scenario_path = str(Path(__file__).parent.parent / "scenarios/DEU_Test-1_1_T-2.xml")

# Load CommonRoad scenario
scenario, pps = CommonRoadFileReader(cr_scenario_path).open()

# Configure simulation and scenario settings
param = CarlaParams()
param.map = cr_scenario_path
param.offscreen_mode = False  # set to false if your system (GPU) is powerful enough
param.vis_type = CustomVis.EGO
param.ego_view.record_video = True

# Initialize CARLA-Interface and execute simulation with the obstacle with ID 0 as ego vehicle
ci = CarlaInterface(param)
ci.replay(scenario, ego_id=6)
