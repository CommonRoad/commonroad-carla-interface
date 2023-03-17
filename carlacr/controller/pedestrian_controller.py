import logging
from typing import Optional
import math
import carla

from carlacr.controller.controller import CarlaController

from commonroad.scenario.state import TraceState

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class AIWalkerControl(CarlaController):
    def control(self, actor: Optional[carla.Actor] = None, state: Optional[TraceState] = None):
        pass

class ManualWalkerControl(CarlaController):
    def control(self, actor: Optional[carla.Actor] = None, state: Optional[TraceState] = None):
        control = carla.WalkerControl()
        control.speed = state.velocity
        rotation = actor.get_transform().rotation
        rotation.yaw = -state.orientation * 180 / math.pi
        control.direction = rotation.get_forward_vector()
        actor.apply_control(control)
