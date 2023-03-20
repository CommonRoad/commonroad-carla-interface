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
    def __init__(self, actor: carla.Actor):
        self._actor = actor
        self._autopilot_enabled = False

    def control(self, state: Optional[TraceState] = None):
        pass

    def register(self, clock: pygame.time.Clock, hud, vis_world):
        pass


class TransformControl(CarlaController):
    def __init__(self, actor: carla.Actor):
        super().__init__(actor)
        self._actor = actor

    def control(self, state: Optional[TraceState] = None):
        transform = create_carla_transform(state, self._actor.get_location().z)
        self._actor.set_transform(transform)
