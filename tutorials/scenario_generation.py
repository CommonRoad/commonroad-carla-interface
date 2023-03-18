import logging
from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.scenario.scenario import Tag

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

# Configure simulation and scenario settings
param = CarlaParams()
#param.map = os.path.dirname(__file__) + "/../maps/four_way_crossing.xodr"
param.map = "Town10HD"
param.vehicle.vehicle_ks_state = False
param.simulation.max_time_step = 120
param.offscreen_mode = False
param.simulation.number_vehicles = 2
param.simulation.number_walkers = 2
param.simulation.osm_mode = True

# Execute scenario generation
ci = CarlaInterface(param)
sc, pps = ci.scenario_generation(ci.create_cr_map())  # generate scenario and convert default map to CommonRoad format

# Store generated scenario
CommonRoadFileWriter(sc, pps,
                     author="TUM-CPS", affiliation="Technical University of Munich", source="CARLA", tags={Tag.URBAN}
                     ).write_to_file(None, OverwriteExistingFile.ALWAYS)
