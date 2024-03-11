import logging
from pathlib import Path

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.scenario.scenario import Tag
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot as plt

from crcarla.carla_interface import CarlaInterface
from crcarla.helper.config import CarlaParams

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

param = CarlaParams()

param.map = "Town10HD"
param.vehicle.vehicle_ks_state = False
param.simulation.max_time_step = 240
param.offscreen_mode = True
param.simulation.number_vehicles = 50
param.simulation.number_walkers = 50

image_path = Path(__file__).parent / "scenario_generation_vis"
if not image_path.exists():
    image_path.mkdir(parents=True, exist_ok=True)
param.visualization.camera_storage_path = str(image_path)

# Execute scenario generation
ci = CarlaInterface(param)
sc, pps = ci.scenario_generation(ci.create_cr_map())

# Store generated scenario
sc.scenario_id.country_id = "ZAM"
sc.scenario_id.map_name = "CARLATown10"
CommonRoadFileWriter(
    sc,
    pps,
    author="Sebastian Maierhofer",
    affiliation="Technical University of Munich",
    source="CARLA",
    tags={Tag.URBAN},
).write_to_file(str(image_path / "ZAM_CARLATown10-1.xml"), OverwriteExistingFile.ALWAYS)

for timeStep in [0, 55]:
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    rnd.draw_params.axis_visible = False
    rnd.draw_params.dynamic_obstacle.trajectory.draw_trajectory = False
    rnd.draw_params.lanelet_network.lanelet.draw_start_and_direction = False
    rnd.draw_params.lanelet_network.lanelet.draw_line_markings = False
    rnd.draw_params.dynamic_obstacle.draw_icon = True
    rnd.draw_params.time_begin = timeStep
    sc.draw(rnd)
    rnd.render()
    plt.ylim((-49, 12))
    plt.xlim((-84, -4.5))
    plt.savefig(str(image_path / f"ZAM_CARLATown10-1-TimeStep{timeStep}.svg"))
