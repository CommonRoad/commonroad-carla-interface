import logging
from pathlib import Path

import carla
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from crdesigner.map_conversion.map_conversion_interface import commonroad_to_opendrive
from crdesigner.ui.gui.controller.animated_viewer.animated_viewer_controller import (
    extract_plot_limits,
)
from matplotlib import pyplot as plt

from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

for cr_scenario_path in Path.glob(Path(__file__).parent / "map_conversion", "*xml"):
    logger.info("Convert and visualize map {}".format(cr_scenario_path.stem))
    or_map_path = Path(__file__).parent / "map_conversion" / f"{cr_scenario_path.stem}.xodr"

    commonroad_to_opendrive(cr_scenario_path, or_map_path)

    # Load CommonRoad scenario and solution file
    scenario, _ = CommonRoadFileReader(cr_scenario_path).open()
    centroid = np.mean(np.concatenate([la.center_vertices for la in scenario.lanelet_network.lanelets]), axis=0)
    plot_limits = extract_plot_limits(scenario.lanelet_network)
    z_axis = np.linalg.norm(np.array([plot_limits[0], plot_limits[2]]) - np.array([plot_limits[1], plot_limits[3]])) / 2

    for obs in scenario.obstacles:
        scenario.remove_obstacle(obs)

    # Configure simulation and scenario settings
    param = CarlaParams()
    param.map = str(or_map_path)
    param.offscreen_mode = True
    param.simulation.max_time_step = 10
    param.simulation.number_vehicles = 1
    param.simulation.number_walkers = 0
    param.visualization.camera_storage_path = str(Path(__file__).parent / "map_conversion")
    param.visualization.camera_transform_bird = carla.Transform(
        carla.Location(z=z_axis, x=centroid[0], y=-centroid[1]), carla.Rotation(pitch=-90.0, yaw=0.0, roll=-90.0)
    )

    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    rnd.draw_params.axis_visible = False
    rnd.draw_params.dynamic_obstacle.trajectory.draw_trajectory = False
    rnd.draw_params.lanelet_network.lanelet.draw_start_and_direction = False
    rnd.draw_params.lanelet_network.lanelet.draw_line_markings = False
    rnd.draw_params.dynamic_obstacle.draw_icon = True
    scenario.draw(rnd)
    rnd.render()
    plt.savefig(str(Path(__file__).parent / "map_conversion" / f"{scenario.scenario_id}.svg"))

    # Initialize CARLA-Interface and execute simulation
    ci = CarlaInterface(param)
    ci.scenario_generation(scenario)
    break
