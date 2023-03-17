import logging
from typing import List, Optional, Union
from abc import ABC

import carla

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleRole, ObstacleType
from commonroad.scenario.trajectory import PMState, KSState, State

from carlacr.helper.config import VehicleParams, PedestrianParams

logger = logging.getLogger(__name__)


class ObstacleInterface(ABC):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle, config: Union[VehicleParams, PedestrianParams]):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        self._carla_id = None
        self._is_spawned = False
        self._config = config
        self._world = None
        self._trajectory = []  # TODO delete later and use cr-io history
        if cr_obstacle is not None:
            self._commonroad_id = cr_obstacle.obstacle_id
            self._time_step = cr_obstacle.initial_state.time_step
            self._cr_base = cr_obstacle
        else:
            self._commonroad_id = None
            self._time_step = None
            self._cr_base: Optional[DynamicObstacle] = None

    def spawn(self, world: carla.World, time_step: int):
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :param config: obstacle-related configuration
        :param physics: if physics should be enabled for the vehicle
        :return: if spawn successful the according CARLA actor else None
        """
        pass

    @property
    def is_spawned(self) -> bool:
        return self._is_spawned

    @property
    def trajectory(self) -> List[Union[PMState, KSState]]:
        return self._trajectory

    @property
    def carla_id(self) -> int:
        return self._carla_id

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
        if self._is_spawned:
            actor = world.get_actor(self._carla_id)
            if actor:
                actor.destroy()
