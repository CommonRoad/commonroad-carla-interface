import logging
from typing import Optional
import math
import carla
import numpy as np

from carlacr.controller.controller import CarlaController

from commonroad.scenario.state import TraceState

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class AIWalkerControl(CarlaController):
    """Interface to CARLA AI controller for walkers."""

    def __init__(self, actor: carla.Actor, final_pos: np.array, max_speed: float):
        """
        Initialization of AI walker controller.

        :param actor: CARLA actor which will be controlled.
        :param final_pos: Goal position of walker.
        :param max_speed: Maximum speed at which the walker should walk.
        """
        super().__init__(actor)
        world = actor.get_world()
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        ai_actor = world.spawn_actor(walker_controller_bp, carla.Transform(), actor)
        ai_actor.start()
        location = carla.Location(x=final_pos[0], y=-final_pos[1], z=0.5)
        ai_actor.go_to_location(location)
        ai_actor.set_max_speed(max_speed)

    def control(self, state: Optional[TraceState] = None):
        """
        Applies CARLA walker AI controller. Nothing to do here since the AI walker works independently.

        :param state: State which should be reached at next time step.
        """


class ManualWalkerControl(CarlaController):
    """Interface to manual walker control of CARLA."""

    def control(self, state: Optional[TraceState] = None):
        """
        Applies CARLA walker controller.

        :param state: State which should be reached at next time step.
        """
        control = carla.WalkerControl()
        control.speed = state.velocity
        rotation = self._actor.get_transform().rotation
        rotation.yaw = -state.orientation * 180 / math.pi
        control.direction = rotation.get_forward_vector()
        self._actor.apply_control(control)
