import os
import logging

from carlacr.interface.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams, CustomVis

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Specify CommonRoad and OpenDRIVE map
or_map_path = "Town10HD"
cr_scenario_path = os.path.dirname(__file__) + "/../scenarios/Town10HD.xml"
#cr_solution_path = os.path.dirname(__file__) + "/solution_PM2:JB1:DEU_Test-1_1_T-1:2020a.xml"

# Load CommonRoad scenario and solution file
scenario, pps = CommonRoadFileReader(cr_scenario_path).open()
#solution = CommonRoadSolutionReader().open(cr_solution_path)

# Configure simulation and scenario settings
param = CarlaParams()
param.map = or_map_path
param.offscreen_mode = True # set to true if your system (GPU) is powerful enough
param.vis_type = CustomVis.EGO # change if your system (GPU) is powerful enough
param.simulation.record_video = True
param.simulation.video_path = os.path.join(os.path.dirname(__file__), "video")

# Initialize CARLA-Interface and execute simulation
ci = CarlaInterface(param)
ci.replay(scenario, ego_id=2964, waypoint_control=True)
