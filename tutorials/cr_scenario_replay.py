import os
import logging
from carlacr.interface.carla_interface import CarlaInterface
from commonroad.common.file_reader import CommonRoadFileReader
from carlacr.helper.config import CarlaParams

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

# Specify CommonRoad and OpenDRIVE map
or_map = os.path.dirname(__file__) + "/../maps/DEU_Test-1_1_T-1.xodr"
cr_map = os.path.dirname(__file__) + "/../scenarios/DEU_Test-1_1_T-1.xml"

# Configure simulation and scenario settings
scenario, _ = CommonRoadFileReader(cr_map).open()
param = CarlaParams()
param.map = or_map
param.offscreen_mode = True # set to false if your system is powerful enough
param.birds_eye_view = True # set to false if your system is powerful enough

# Execute scenario simulation
ci = CarlaInterface(param)
ci.replay(scenario)
