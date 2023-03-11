from typing import List, Optional
import logging
import carla

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
        if self._waypoint_control:
            self.control(self.cr_obstacle.state_at_time(self._time_step))
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
            return [carla.Location(x=state.position[0],  y=-state.position[1], z=0.5)
                    for state in self._cr_base.prediction.trajectory.state_list]