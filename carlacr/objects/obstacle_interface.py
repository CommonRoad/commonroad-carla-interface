import logging
from typing import List, Optional, Union
from abc import ABC

import carla

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleRole, ObstacleType
from commonroad.scenario.trajectory import PMState, KSState

from carlacr.helper.config import VehicleParams, PedestrianParams

logger = logging.getLogger(__name__)


class ObstacleInterface(ABC):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, config: Union[VehicleParams, PedestrianParams], cr_obstacle: DynamicObstacle,
                 actor: Optional[Union[carla.Vehicle, carla.Walker]]):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        self._actor = actor
        self._config = config
        self._controller = None
        self._trajectory = []  # TODO delete later and use cr-io history
        self._cr_base = cr_obstacle

    def _spawn(self, world: carla.World, time_step: int):
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :param config: obstacle-related configuration
        :param physics: if physics should be enabled for the vehicle
        :return: if spawn successful the according CARLA actor else None
        """
        pass

    @property
    def spawned(self) -> bool:
        return self._actor is not None

    @property
    def trajectory(self) -> List[Union[PMState, KSState]]:
        return self._trajectory

    @property
    def actor(self) -> carla.Actor:
        return self._actor

    @property
    def cr_obstacle(self) -> DynamicObstacle:
        return self._cr_base

    def get_role(self) -> ObstacleRole:
        return self._cr_base.obstacle_role

    def get_type(self) -> ObstacleType:
        return self._cr_base.obstacle_type

    def state_at_time_step(self, time_step: int):
        return self._cr_base.state_at_time(time_step)

    @property
    def control_type(self):
        return self._config.controller_type

    def tick(self, clock, world: carla.World, tm: carla.TrafficManager):
        pass

    def destroy_carla_obstacle(self, world):
        """
        Destroys vehicle in CARLA.

        :param world: the CARLA world object
        """
        if self._actor is not None:
            self._actor.destroy()
