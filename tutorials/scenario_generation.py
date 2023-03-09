import logging
from carlacr.interface.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams

from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.scenario.scenario import Tag

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

# Configure scenario settings
param = CarlaParams()
param.map = "Town01"  # use CARLA default map
param.obstacle.vehicle_ks_state = False
param.simulation.max_time_step = 60

# Execute scenario generation
ci = CarlaInterface(param)
sc, pps = ci.scenario_generation(ci.create_cr_map())  # generate scenario and convert default map to CommonRoad format

# Store generated scenario
CommonRoadFileWriter(sc, pps,
                     author="TUM-CPS", affiliation="Technical University of Munich", source="CARLA", tags={Tag.URBAN}
                     ).write_to_file()
