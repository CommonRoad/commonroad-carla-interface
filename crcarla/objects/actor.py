from abc import ABC
from typing import List, Optional, Union

import carla
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.state import KSState, PMState, ExtendedPMState

from crcarla.helper.config import PedestrianParams, VehicleParams


class ActorInterface(ABC):
    """Abstract interface between CARLA actor and CommonRoad obstacles."""

    def __init__(
        self,
        cr_obstacle: DynamicObstacle,
        world: carla.World,
        tm: Optional[carla.TrafficManager],
        actor: Union[carla.Vehicle, carla.Walker],
        config: Union[VehicleParams, PedestrianParams],
    ):
        """
        Initializer of actor interface.

        :param cr_obstacle: CommonRoad obstacle corresponding to the actor.
        :param world: CARLA world
        :param tm: CARLA traffic manager.
        :param actor: New CARLA actor. None if actor is not spawned yet.
        :param config: Vehicle or pedestrian config parameters.
        """
        self._actor = actor
        self._world = world
        self._tm = tm
        self._config = config
        self._controller = None
        self._cr_obstacle = cr_obstacle

    def _spawn(self, time_step: int):
        """
        Tries to spawn the vehicle (incl. lights if supported) in the given CARLA world.
        Concrete implementation is actor type dependent.

        :param time_step: Current time step.

        """

    def _init_controller(self):
        """Initializes CARLA controller used for. Concrete implementation is actor type specific."""

    @property
    def spawned(self) -> bool:
        """
        Getter for spawned.

        :return: Boolean indicating whether actor is spawned.
        """
        return self._actor is not None

    @property
    def trajectory(self) -> List[Union[PMState, KSState, ExtendedPMState]]:
        """
        Getter for trajectory.

        :return: List of state elements.
        """
        # check if static obstacle without history
        if not hasattr(self._cr_obstacle, "history"):
            return []
        return self.cr_obstacle.history

    @property
    def actor(self) -> carla.Actor:
        """
        Getter for actor.

        :return: CARLA actor object.
        """
        return self._actor

    @property
    def cr_obstacle(self) -> DynamicObstacle:
        """
        Getter for CommonRoad obstacle.

        :return: Dynamic obstacle object.
        """
        return self._cr_obstacle

    @property
    def control_type(self):
        """
        Getter for used control type.

        :return: CARLA control type.
        """
        return self._config.carla_controller_type

    def tick(self, time_step: int):
        """
        Performs one-step planning/simulation. Concrete implementation is actor type dependent.

        :param time_step: Current time step.
        """

    def destroy_carla_obstacle(self):
        """Destroys vehicle in CARLA."""
        if self._actor is not None:
            self._actor.destroy()
