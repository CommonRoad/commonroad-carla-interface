import logging
from typing import Optional
from abc import ABC
import math
import carla
import pygame

# from carlacr.visualization.ego_view import HUD3D, World3D
# from carlacr.visualization.birds_eye_view import HUD2D, World2D

from commonroad.scenario.state import TraceState

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def create_carla_transform(state: TraceState, z_position: float = 0.5):
    transform = carla.Transform(
            carla.Location(x=state.position[0], y=-state.position[1], z=z_position),
            carla.Rotation(yaw=(-(180 * state.orientation) / math.pi)))
    return transform

class CarlaController(ABC):
    """Interface for CARLA controllers."""
    def __init__(self, actor: carla.Actor):
        """
        Initialization of general CARLA controller properties.

        :param actor: CARLA actor which will be controlled.
        """
        self._actor = actor
        self._autopilot_enabled = False

    def control(self, state: Optional[TraceState] = None):
        """
        Computes and applies vehicle/walker input. Concrete implementation in corresponding controller.

        :param state: State which should be reached at next time step.
        """
        pass

    def register(self, clock: pygame.time.Clock, hud, vis_world):
        """
        Registers some external information for controller.
        For example, if some controller-specific information should be rendered.

        TODO parameters
        """
        pass


class TransformControl(CarlaController):
    """Controller which translates and rotates actor based on CommonRoad state."""
    def __init__(self, actor: carla.Actor):
        """
        Initialization of transform controller.

        :param actor: CARLA actor which will be controlled.
        """
        super().__init__(actor)

    def control(self, state: Optional[TraceState] = None):
        """
        Computes and applies CARLA transform by translation and rotation.

        :param state: State which should be reached at next time step.
        """
        transform = create_carla_transform(state, self._actor.get_location().z)
        self._actor.set_transform(transform)

