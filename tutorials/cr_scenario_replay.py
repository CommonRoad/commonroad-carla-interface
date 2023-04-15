import os
import logging

from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams, CustomVis, VehicleControlType, PedestrianControlType

from commonroad.common.file_reader import CommonRoadFileReader

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Specify CommonRoad and OpenDRIVE map
# or_map_path = "Town10HD"
or_map_path = os.path.dirname(__file__) + "/../maps/four_way_crossing.xodr"
# cr_scenario_path = os.path.dirname(__file__) + "/../scenarios/Town10HD.xml"
cr_scenario_path = os.path.dirname(__file__) + "/../scenarios/four_way_crossing_Modi.xml"
#cr_solution_path = os.path.dirname(__file__) + "/solution_PM2:JB1:DEU_Test-1_1_T-1:2020a.xml"

# Load CommonRoad scenario and solution file
scenario, pps = CommonRoadFileReader(cr_scenario_path).open()
#solution = CommonRoadSolutionReader().open(cr_solution_path)

# Configure simulation and scenario settings
param = CarlaParams()
param.map = or_map_path
param.offscreen_mode = False # set to true if your system (GPU) is powerful enough
param.vis_type = CustomVis.NONE # change if your system (GPU) is powerful enough
param.vehicle.carla_controller_type = VehicleControlType.PID
param.pedestrian.controller_type = PedestrianControlType.AI
# param.simulation.record_video = False
# param.simulation.video_path = os.path.join(os.path.dirname(__file__), "video")

# Initialize CARLA-Interface and execute simulation
ci = CarlaInterface(param)
ci.replay(scenario)
