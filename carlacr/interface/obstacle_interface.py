import logging
import numpy as np

import carla

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleRole
from commonroad.scenario.trajectory import State

from carlacr.helper.config import ObstacleParams

logger = logging.getLogger(__name__)


def create_carla_transform(state: State, z_position: float = 0.5):
    transform = carla.Transform(
            carla.Location(x=state.position[0], y=-state.position[1], z=z_position),
            carla.Rotation(yaw=(-(180 * state.orientation) / np.pi)))
    return transform


class ObstacleInterface:
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle, config: ObstacleParams = ObstacleParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        self._commonroad_id = cr_obstacle.obstacle_id
        self._carla_id = None
        self._is_spawned = False
        self._spawn_timestep = cr_obstacle.initial_state.time_step
        self._cr_base = cr_obstacle
        self._config = config

    def spawn(self, world: carla.World) -> carla.Actor:
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :param config: obstacle-related configuration
        :param physics: if physics should be enabled for the vehicle
        :return: if spawn successful the according CARLA actor else None
        """

        pass

    def update_position_by_time(self, world: carla.World, state: State):
        """
        Tries to update the position of the obstacle and sets lights.

        :param world: the CARLA world object
        :param state:state at the time step
        """
        try:
            if self._is_spawned & (self._cr_base.obstacle_role == ObstacleRole.DYNAMIC) and \
                    (self._cr_base.prediction.trajectory is not None):
                actor = world.get_actor(self._carla_id)
                if actor:
                    transform = create_carla_transform(state, actor.get_location().z)
                    actor.set_transform(transform)
                else:
                    logger.debug("Could not find actor")
        except Exception as e:
            logger.error("Error while updating position")
            raise e

    def destroy_carla_obstacle(self, world):
        """
        Destroys vehicle in CARLA.

        :param world: the CARLA world object
        """
        if self._is_spawned:
            actor = world.get_actor(self._carla_id)
            if actor:
                actor.destroy()

    def __str__(self):
        """Gives a description as String of the characteristics of the Obstacle."""
        resp = f"CommonRoad ID: {self._commonroad_id}\n"
        resp += f"CARLA ID: {self._carla_id}\n"
        resp += f"is_spawned: {self._is_spawned}\n"
        resp += f"spawn_timestep: {self._spawn_timestep}\n"
        resp += f"trajectory: {{{self._cr_base.prediction.trajectory}\n}}\n"
        resp += f"size: {self._cr_base.obstacle_shape.length}, {self._cr_base.obstacle_shape.width}\n"
        resp += f"role: {self._cr_base.obstacle_role}"
        resp += f"type: {self._cr_base.obstacle_type}\n"
        return resp
