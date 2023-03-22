import logging
from typing import Optional
from abc import ABC
import math
import carla

from commonroad.scenario.state import TraceState

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def create_carla_transform(state: TraceState, z_position: float = 0.5) -> carla.Transform:
    """
    Computes CARLA transform given CommonRoad state and a z-position.

    :param state: CommonRoad state.
    :param z_position: z-position which transform object should have.
    :return: CARLA transform.
    """
    transform = carla.Transform(carla.Location(x=state.position[0], y=-state.position[1], z=z_position),
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


class TransformControl(CarlaController):
    """Controller which translates and rotates actor based on CommonRoad state."""

    def control(self, state: Optional[TraceState] = None):
        """
        Computes and applies CARLA transform by translation and rotation.

        :param state: State which should be reached at next time step.
        """
        transform = create_carla_transform(state, self._actor.get_location().z)
        self._actor.set_transform(transform)
