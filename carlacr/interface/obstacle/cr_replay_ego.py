from typing import List, Optional
import logging
import carla
import math

from carlacr.interface.obstacle.ego_interface import EgoInterface
from carlacr.helper.config import ObstacleParams

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleRole

logger = logging.getLogger(__name__)


class CommonRoadObstacleInterface(EgoInterface):

    def __init__(self, cr_obstacle: Optional[DynamicObstacle], waypoint_control: bool = False,
                 config: ObstacleParams = ObstacleParams()):
        """Initializes input member variables when instance is created."""
        super().__init__(cr_obstacle, config)
        self._waypoint_control = waypoint_control
        self._time_step = cr_obstacle.initial_state.time_step

    def tick(self, clock, world, tm):
        if not self._is_spawned:
            self.spawn(world, self._time_step, tm)
        cur_state = self.cr_obstacle.state_at_time(self._time_step)
        if self._waypoint_control:
            self.control(cur_state)
        else:
            if hasattr(tm, "set_desired_speed"):
                actor = world.get_actor(self._carla_id)
                if hasattr(cur_state, "velocity_y"):
                    vel = math.sqrt(cur_state.velocity ** 2 + cur_state.velocity_y ** 2)
                    tm.set_desired_speed(actor, vel)
                else:
                    tm.set_desired_speed(actor, cur_state.velocity)
        self._time_step += 1

    def spawn(self, world: carla.World, time_step: int, tm: carla.TrafficManager = None):
        super().spawn(world, time_step)
        if not self._waypoint_control:
            if self.get_role() == ObstacleRole.DYNAMIC:
                tm.set_path(world.get_actor(self._carla_id), self._get_path())

    def _get_path(self) -> List[carla.Location]:
        if self._cr_base.obstacle_role is not ObstacleRole.DYNAMIC:
            return [carla.Location(x=self._cr_base.initial_state.position[0],
                                   y=-self._cr_base.initial_state.position[1],
                                   z=0.5)]
        else:
            path = []
            for time_step in range(0, len(self._cr_base.prediction.trajectory.state_list), self._config.path_sampling):
                state = self._cr_base.prediction.trajectory.state_list[time_step]
                path.append(carla.Location(x=state.position[0],  y=-state.position[1], z=0.5))
            if len(self._cr_base.prediction.trajectory.state_list) % self._config.path_sampling != 0:
                state = self._cr_base.prediction.trajectory.state_list[-1]
                path.append(carla.Location(x=state.position[0], y=-state.position[1], z=0.5))
            return path