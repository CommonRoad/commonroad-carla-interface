#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

from typing import List, Tuple

import carla
import pygame

from carlacr.helper.config import EgoViewParams
from carlacr.visualization.canvas.canvas_controller import CanvasController
from carlacr.visualization.common import get_actor_display_name, sort_vehicles_by_dist
from carlacr.visualization.sensors.sensor_controller import SensorController
from carlacr.visualization.visualization_base import VisualizationBase
from carlacr.visualization.visualization_tools.visualization_tools_controller import (
    VisualizationToolsController,
)


class Visualization3D(VisualizationBase):
    """
    Class for 3D ego view visualization that contains all the
    information of a CARLA world that is running on the server side.

    This class also controls all VisualizationBase instances.
    """

    def __init__(self, carla_world: carla.World, config: EgoViewParams, ego_vehicle: carla.Vehicle):
        """
        Initialization of 3D world visualization.

        :param carla_world: CARLA world.
        :param config: Simulation parameters.
        :param ego_vehicle: Ego vehicle actor.
        """
        super().__init__()  # must have for VisualizationBase inheriting __init__()

        # Store parameters
        self.carla_world = carla_world
        self.config = config
        self._ego_vehicle = ego_vehicle
        self._vehicles = []
        self._vehicles_by_dist: List[Tuple[carla.Vehicle, float]] = None

        self.canvas_controller = CanvasController(self)
        self.sensor_controller = SensorController(self)
        self.vis_tool_controller = VisualizationToolsController(self)

        # store information to display
        self.server_fps = 0
        self.client_fps = 0

        self._server_clock = pygame.time.Clock()
        self.frame = 0
        self.simulation_time = 0

        self._on_world_tick_ID = self.carla_world.on_tick(self._on_world_tick)

        # Restart/reinitialize the visualization
        self.restart()

    # ==========================================
    # Overwrite pipeline from VisualizationBase
    # ==========================================

    def restart(self):
        super().restart()
        instances = VisualizationBase.get_instances()
        for obj in instances:
            if self is not obj:
                obj.restart()  # all VisualizationBase objects

        self.canvas_controller.notify(get_actor_display_name(self._ego_vehicle))
        self.canvas_controller.notify("Press 'H' or '?' for help.", t_ms=4000)

        if self.config.sync:
            self.carla_world.tick()
        else:
            self.carla_world.wait_for_tick()

    def tick(self, clock: pygame.time.Clock):
        """
        Performs a tick of the HUD.

        :param clock: Pygame clock.
        """
        super().tick(clock)
        self._vehicles = self.__get_vehicles()
        self._vehicles_by_dist = sort_vehicles_by_dist(self._vehicles, self.ego_vehicle, max_dist=0.0)
        self.client_fps = clock.get_fps()

        for obj in VisualizationBase.get_instances():
            if self is not obj:
                obj.tick(clock)  # all VisualizationBase objects

    def render(self, display: pygame.surface.Surface):
        """
        Calls the renderer of the camera manager and HUD.

        :param display: Pygame display for rendering.
        """
        # control VisualizationBase class
        super().render(display)
        for obj in VisualizationBase.get_instances():
            if self is not obj:
                obj.render(display)  # all VisualizationBase objects

    def destroy(self):
        """Destroys the sensors and the ego vehicle."""
        self.carla_world.remove_on_tick(self._on_world_tick_ID)

        super().destroy()
        for obj in VisualizationBase.get_instances():
            if self is not obj:
                obj.destroy()  # all VisualizationBase objects

        if self._ego_vehicle is not None:
            self._ego_vehicle.destroy()

    # ==========================================
    # Properties
    # ==========================================

    @property
    def ego_vehicle(self) -> carla.Vehicle:
        """
        Getter for the ego vehicle.

        :return: CARLA ego vehicle.
        """
        return self._ego_vehicle

    @property
    def vehicles(self) -> carla.ActorList:
        """
        Getter for all vehicles in the world.
        (once per tick)

        :return: List of vehicles.
        """
        if not self._vehicles:
            self._vehicles = self.__get_vehicles()
        return self._vehicles

    @property
    def vehicles_by_dist(self) -> List[Tuple[carla.Vehicle, float]]:
        """
        Getter for all vehicles in the world.
        (once per tick)

        :return: List of vehicles.
        """
        return self._vehicles_by_dist

    # ==========================================
    # Private
    # ==========================================

    def _on_world_tick(self, timestamp: carla.Timestamp):
        """
        Callback for CARLA world tick.

        :param timestamp: CARLA world snapshot timestamp.
        """
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def __get_vehicles(self) -> carla.ActorList:
        return self.carla_world.get_actors().filter(self.config.object_filter)
