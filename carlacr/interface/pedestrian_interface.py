import logging

import carla

from commonroad.scenario.obstacle import Obstacle

from carlacr.helper.config import ObstacleParams
from carlacr.interface.obstacle_interface import ObstacleInterface

logger = logging.getLogger(__name__)


class PedestrianInterface(ObstacleInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: Obstacle):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle)

    def spawn(self, world: carla.World, config: ObstacleParams) -> carla.Actor:
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :param config: obstacle-related configuration
        :return: if spawn successful the according CARLA actor else None
        """
        transform = self.init_transform()
        obstacle_blueprint_walker = world.get_blueprint_library().find('walker.pedestrian.0002')
        try:
            obstacle = world.spawn_actor(obstacle_blueprint_walker, transform)  # parent_walker
            if obstacle:
                obstacle.set_simulate_physics(config.physics)
                logger.debug("Spawn successful: CR-ID %s CARLA-ID %s", self.commonroad_id, obstacle.id)
                self.carla_id = obstacle.id
                self.is_spawned = True
        except Exception as e:
            logger.error(f"Error while spawning PEDESTRIAN: {e}")
            raise e
