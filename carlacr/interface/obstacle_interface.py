import logging
import numpy as np

import carla

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleRole
from commonroad.scenario.trajectory import State

from carlacr.helper.config import ObstacleParams

logger = logging.getLogger(__name__)


class ObstacleInterface:
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        self.commonroad_id = cr_obstacle.obstacle_id
        self.carla_id = None
        self.is_spawned = False
        self.spawn_timestep = cr_obstacle.initial_state.time_step
        self.cr_base = cr_obstacle

    def spawn(self, world: carla.World, config: ObstacleParams) -> carla.Actor:
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :param config: obstacle-related configuration
        :param physics: if physics should be enabled for the vehicle
        :return: if spawn successful the according CARLA actor else None
        """

        pass

    def transform(self, state: State, z_position: float = 0.5):
        transform = carla.Transform(
                carla.Location(x=state.position[0], y=-state.position[1], z=z_position),
                carla.Rotation(yaw=(-(180 * state.orientation) / np.pi)))
        return transform

    def update_position_by_time(self, world: carla.World, state: State):
        """
        Tries to update the position of the obstacle and sets lights.

        :param world: the CARLA world object
        :param state:state at the time step
        """
        try:
            if self.is_spawned & (self.cr_base.obstacle_role == ObstacleRole.DYNAMIC) and \
                    (self.cr_base.prediction.trajectory is not None):
                actor = world.get_actor(self.carla_id)
                if actor:
                    transform = self.transform(state, actor.get_location().z)
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
        if self.is_spawned:
            actor = world.get_actor(self.carla_id)
            if actor:
                actor.destroy()

    def __str__(self):
        """Gives a description as String of the characteristics of the Obstacle."""
        resp = f"commonroad_id: {self.commonroad_id}\n"
        resp += f"carla_id: {self.carla_id}\n"
        resp += f"is_spawned: {self.is_spawned}\n"
        resp += f"spawn_timestep: {self.spawn_timestep}\n"
        resp += f"next_update_time: {self.next_update_time}\n"
        resp += f"trajectory: {{{self.trajectory}\n}}\n"
        resp += f"size: {self.size}\n"
        resp += f"role: {self.role}"
        resp += f"type: {self.type}\n"
        return resp
