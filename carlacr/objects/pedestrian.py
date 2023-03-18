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
                 actor: Optional[carla.Walker] = None, config: PedestrianParams = PedestrianParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(config, cr_obstacle, actor)
        self._controller = None

    def _init_controller(self):
        if self._config.controller_type is PedestrianControlType.TRANSFORM:
            self._controller = TransformControl(self._actor)
        elif self._config.controller_type is PedestrianControlType.AI:
            self._controller = AIWalkerControl(self._actor)
        elif self._config.controller_type is PedestrianControlType.WALKER:
            self._controller = ManualWalkerControl(self._actor)


    def _spawn(self, world: carla.World, time_step: int):
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world and returns the spawned vehicle.

        :param world: the CARLA world object
        :return: if spawn successful the according CARLA actor else None
        """
        self._world = world
        if time_step != self._cr_base.initial_state.time_step or self.spawned:
            return
        transform = create_carla_transform(self._cr_base.initial_state)
        obstacle_blueprint_walker = world.get_blueprint_library().find('walker.pedestrian.0002')
        try:
            actor = world.spawn_actor(obstacle_blueprint_walker, transform)  # parent_walker
            if actor:
                actor.set_simulate_physics(self._config.physics)
                logger.debug("Spawn successful: CR-ID %s CARLA-ID %s", self._cr_base.obstacle_id, actor.id)
        except Exception as e:
            logger.error(f"Error while spawning PEDESTRIAN: {e}")
            raise e

        if self._config.controller_type is PedestrianControlType.AI:
            walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
            ai_actor = world.spawn_actor(walker_controller_bp, carla.Transform(), actor)
            ai_actor.start()
            position = self._cr_base.prediction.trajectory.final_state.position
            location = carla.Location(x=position[0], y=-position[1], z=0.5)
            ai_actor.go_to_location(location)
            ai_actor.set_max_speed(max([state.velocity for state in self._cr_base.prediction.trajectory.state_list]))
        self._actor = actor

    def tick(self, world: carla.World, time_step: int, tm: Optional[carla.TrafficManager] = None):
        if not self.spawned:
            self._spawn(world, time_step)
            self._init_controller()
        else:
            self._controller.control(self.cr_obstacle.state_at_time(time_step))