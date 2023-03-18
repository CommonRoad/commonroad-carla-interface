import logging
from typing import Optional
import carla

from commonroad.scenario.obstacle import DynamicObstacle

from carlacr.helper.config import PedestrianParams, PedestrianControlType
from carlacr.objects.obstacle_interface import ObstacleInterface
from carlacr.controller.controller import create_carla_transform, TransformControl
from carlacr.controller.pedestrian_controller import AIWalkerControl, ManualWalkerControl

logger = logging.getLogger(__name__)


class PedestrianInterface(ObstacleInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle,
                 carla_id: Optional[int] = None, config: PedestrianParams = PedestrianParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle, config)
        self._carla_id = carla_id
        self._ai_controller_id = None
        self._controller = self._init_controller()

    def _init_controller(self):
        if self._config.controller_type is PedestrianControlType.TRANSFORM:
            return TransformControl()
        elif self._config.controller_type is PedestrianControlType.AI:
            return AIWalkerControl()
        elif self._config.controller_type is PedestrianControlType.WALKER:
            return ManualWalkerControl()


    def _spawn(self, world: carla.World, time_step: int):
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :return: if spawn successful the according CARLA actor else None
        """
        self._world = world
        if time_step != self._cr_base.initial_state.time_step or self._is_spawned:
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

        if self._config.controller_type is PedestrianControlType.AI:
            walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
            actor = world.spawn_actor(walker_controller_bp, carla.Transform(), self._world.get_actor(self._carla_id))
            self._ai_controller_id = actor.id
            actor.start()
            position = self._cr_base.prediction.trajectory.final_state.position
            location = carla.Location(x=position[0], y=-position[1], z=0.5)
            actor.go_to_location(location)
            actor.set_max_speed(max([state.velocity for state in self._cr_base.prediction.trajectory.state_list]))

    def tick(self, world: carla.World, tm: carla.TrafficManager, time_step: int):
        if not self._is_spawned:
            self._spawn(world, time_step)
        else:
            self._controller.control(world.get_actor(self._carla_id), self.cr_obstacle.state_at_time(time_step))