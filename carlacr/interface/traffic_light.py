import logging
import numpy as np
from typing import Optional

import carla

from commonroad.scenario.traffic_sign import TrafficLight

logger = logging.getLogger(__name__)


def create_carla_transform(position: np.array, orientation: float, z_position: float = 2.0):
    transform = carla.Transform(
            carla.Location(x=position[0], y=-position[1], z=z_position),
            carla.Rotation(yaw=(-(180 * orientation) / np.pi)))
    return transform


class CarlaTrafficLight:
    def __init__(self, cr_tl: TrafficLight, config):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        self._carla_id = None
        self._config = config
        self._world = None
        self._trajectory = []  # TODO delete later and use cr-io history
        if cr_tl is not None:
            self._commonroad_id = cr_tl.traffic_light_id
            self._cr_base = cr_tl
        else:
            self._commonroad_id = None
            self._cr_base: Optional[TrafficLight] = None
