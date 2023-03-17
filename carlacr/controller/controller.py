import logging
from typing import Optional
from abc import ABC
import math
import carla

from commonroad.scenario.state import TraceState

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def create_carla_transform(state: TraceState, z_position: float = 0.5):
    transform = carla.Transform(
            carla.Location(x=state.position[0], y=-state.position[1], z=z_position),
            carla.Rotation(yaw=(-(180 * state.orientation) / math.pi)))
    return transform

class CarlaController(ABC):
    def __init__(self):
        self._autopilot_enabled = False
    def control(self, actor: Optional[carla.Actor] = None, state: Optional[TraceState] = None):
        pass


class TransformControl(CarlaController):
    def control(self, actor: Optional[carla.Actor] = None, state: Optional[TraceState] = None):
        transform = create_carla_transform(state, actor.get_location().z)
        actor.set_transform(transform)

