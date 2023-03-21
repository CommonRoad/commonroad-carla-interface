import logging
from typing import Optional
import carla

from commonroad.scenario.obstacle import DynamicObstacle

from carlacr.helper.config import PedestrianParams, PedestrianControlType
from carlacr.objects.actor import ActorInterface
from carlacr.controller.controller import create_carla_transform, TransformControl
from carlacr.controller.pedestrian_controller import AIWalkerControl, ManualWalkerControl

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class PedestrianInterface(ActorInterface):
    """Interface between CARLA walker and CommonRoad pedestrian."""

    def __init__(self, cr_obstacle: DynamicObstacle, world: carla.World, tm: Optional[carla.TrafficManager],
                 actor: Optional[carla.Walker] = None, config: PedestrianParams = PedestrianParams()):
        """
        Initializer of pedestrian interface.

        :param cr_obstacle: CommonRoad obstacle corresponding to the actor.
        :param world: CARLA world
        :param tm: CARLA traffic manager.
        :param actor: New CARLA actor. None if actor is not spawned yet.
        :param config: Vehicle or pedestrian config parameters.
        """
        super().__init__(cr_obstacle, world, tm, actor, config)

    def _init_controller(self):
        """Initializes CARLA pedestrian controller used for walker."""
        if self._config.controller_type is PedestrianControlType.TRANSFORM:
            self._controller = TransformControl(self._actor)
        elif self._config.controller_type is PedestrianControlType.AI:
            self._controller = \
                AIWalkerControl(self._actor, self._cr_obstacle.prediction.trajectory.final_state.position,
                                max([state.velocity for state in self._cr_obstacle.prediction.trajectory.state_list]))
        elif self._config.controller_type is PedestrianControlType.WALKER:
            self._controller = ManualWalkerControl(self._actor)

    def _spawn(self, time_step: int):
        """
        Tries to spawn the walker in the given CARLA world at the provided time step.

        :param time_step: Time step at which CARLA walker should be spawned.
        """
        if time_step != self._cr_obstacle.initial_state.time_step or self.spawned:
            return
        transform = create_carla_transform(self._cr_obstacle.initial_state)
        obstacle_blueprint_walker = self._world.get_blueprint_library().find('walker.pedestrian.0002')
        try:
            actor = self._world.spawn_actor(obstacle_blueprint_walker, transform)  # parent_walker
            if actor:
                actor.set_simulate_physics(self._config.physics)
                logger.debug("Spawn successful: CR-ID %s CARLA-ID %s", self._cr_obstacle.obstacle_id, actor.id)
        except Exception as e:
            logger.error(f"Error while spawning PEDESTRIAN: {e}")
            raise e

        self._actor = actor

    def tick(self, time_step: int):
        """
        Performs one-step planning/simulation. If actor is not spawned yet, it will be spawned.

        :param time_step: Current time step.
        """
        if not self.spawned:
            self._spawn(time_step)
            self._init_controller()
        else:
            self._controller.control(self.cr_obstacle.state_at_time(time_step))
