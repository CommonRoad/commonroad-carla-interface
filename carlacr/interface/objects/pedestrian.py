import logging
import math
from typing import Optional
import carla

from commonroad.scenario.obstacle import DynamicObstacle

from carlacr.helper.config import ObstacleParams
from carlacr.interface.objects.obstacle_interface import ObstacleInterface
from carlacr.interface.controller.controller import create_carla_transform

logger = logging.getLogger(__name__)


class PedestrianInterface(ObstacleInterface):
    """One to one representation of a CommonRoad obstacle to be worked with in CARLA."""

    def __init__(self, cr_obstacle: DynamicObstacle, spawned: bool = False,
                 carla_id: Optional[int] = None, waypoint_control: bool = False, config: ObstacleParams = ObstacleParams()):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        super().__init__(cr_obstacle, config)
        self._carla_id = carla_id
        self._is_spawned = spawned
        self._waypoint_control = waypoint_control
        self._time_step = cr_obstacle.initial_state.time_step
        self._ai_controller_id = None

    def spawn(self, world: carla.World, time_step: int):
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

        if not self._waypoint_control:
            walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
            actor = world.spawn_actor(walker_controller_bp, carla.Transform(), self._world.get_actor(self._carla_id))
            self._ai_controller_id = actor.id
            actor.start()
            position = self._cr_base.prediction.trajectory.final_state.position
            location = carla.Location(x=position[0], y=-position[1], z=0.5)
            actor.go_to_location(location)
            actor.set_max_speed(max([state.velocity for state in self._cr_base.prediction.trajectory.state_list]))

    def tick(self, world):
        if not self._is_spawned:
            self.spawn(world, self._time_step)
        cur_state = self.cr_obstacle.state_at_time(self._time_step)
        if self._waypoint_control:
            control = carla.WalkerControl()
            control.speed = cur_state.velocity
            rotation = world.get_actor(self._carla_id).get_transform().rotation
            rotation.yaw = -cur_state.orientation * 180 / math.pi
            control.direction = rotation.get_forward_vector()
            self._world.get_actor(self._carla_id).apply_control(control)
        self._time_step += 1