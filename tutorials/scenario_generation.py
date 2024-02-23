from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.scenario.scenario import Tag

from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams

# Configure simulation and scenario settings
param = CarlaParams()
param.map = "Town10HD"
param.vehicle.vehicle_ks_state = False
param.simulation.max_time_step = 120
param.offscreen_mode = True
param.simulation.number_vehicles = 5
param.simulation.number_walkers = 5
param.simulation.tm.global_lane_offset = 80
param.simulation.tm.ignore_vehicles_percentage = 20

# Execute scenario generation
ci = CarlaInterface(param)
sc, pps = ci.scenario_generation(ci.create_cr_map())  # generate scenario and convert default map to CommonRoad format

# Store generated scenario
CommonRoadFileWriter(
    sc, pps, author="TUM-CPS", affiliation="Technical University of Munich", source="CARLA", tags={Tag.URBAN}
).write_to_file(None, OverwriteExistingFile.ALWAYS)
