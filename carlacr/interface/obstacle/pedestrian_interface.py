import logging
from typing import Optional
import carla

from commonroad.scenario.obstacle import DynamicObstacle

from carlacr.helper.config import ObstacleParams
from carlacr.interface.obstacle.obstacle_interface import ObstacleInterface, create_carla_transform

logger = logging.getLogger(__name__)


class PedestrianInterface(ObstacleInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle, spawned: bool = False,
                 carla_id: Optional[int] = None, config: ObstacleParams = ObstacleParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle, config)

    def spawn(self, world: carla.World, time_step: int):
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :return: if spawn successful the according CARLA actor else None
        """
        self._world = world
        if time_step != self._cr_base.initial_state.time_step:
            return
        transform = create_carla_transform(self._cr_base.initial_state)
        obstacle_blueprint_walker = world.get_blueprint_library().find('walker.pedestrian.0002')
        try:
            obstacle = world.spawn_actor(obstacle_blueprint_walker, transform)  # parent_walker
            if obstacle:
                obstacle.set_simulate_physics(self._config.physics)
                logger.debug("Spawn successful: CR-ID %s CARLA-ID %s", self._commonroad_id, obstacle.id)
                self._carla_id = obstacle.id
                self._is_spawned = True
        except Exception as e:
            logger.error(f"Error while spawning PEDESTRIAN: {e}")
            raise e
