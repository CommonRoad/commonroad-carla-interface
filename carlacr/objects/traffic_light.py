import logging
import numpy as np
from typing import Optional, List
import math

import carla

from commonroad.scenario.traffic_sign import TrafficLight, TrafficLightState, TrafficLightCycleElement

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def _match_carla_traffic_light_state_to_cr(carla_state: carla.TrafficLightState) -> TrafficLightState:
    if carla_state == carla.TrafficLightState.Green:
        return TrafficLightState.GREEN
    if carla_state == carla.TrafficLightState.Red:
        return TrafficLightState.RED
    if carla_state == carla.TrafficLightState.Yellow:
        return TrafficLightState.YELLOW
    if carla_state == carla.TrafficLightState.Off:
        return TrafficLightState.INACTIVE
    return TrafficLightState.RED


def _match_cr_traffic_light_state_to_carla(cr_state: TrafficLightState) -> carla.TrafficLightState:
    if cr_state == TrafficLightState.GREEN:
        return carla.TrafficLightState.Green
    if cr_state == TrafficLightState.RED:
        return carla.TrafficLightState.Red
    if cr_state == TrafficLightState.YELLOW:
        return carla.TrafficLightState.Yellow
    if cr_state == TrafficLightState.INACTIVE:
        return carla.TrafficLightState.Off
    if cr_state == TrafficLightState.RED_YELLOW:
        return carla.TrafficLightState.Yellow
    return carla.TrafficLightState.Red


class CarlaTrafficLight:
    """Interface between CARLA traffic light and CommonRoad traffic light"""

    def __init__(self, actor: carla.TrafficLight, cr_tl: Optional[TrafficLight] = None):
        """
        Initializer carla traffic light interface.

        :param actor: CARLA traffic light.
        :param cr_tl: CommonRoad traffic light object.
        """
        self._actor = actor
        position = actor.get_location()
        self._carla_position = np.array([position.x, position.y, position.z])
        self._signal_profile: List[TrafficLightState] = []
        self._cr_tl = cr_tl
        self._set_initial_color(actor.state)

    def _set_initial_color(self, color: carla.TrafficLightState):
        """
        Setter for initial traffic light state.

        :param color: Current CARLA traffic light color.
        """
        if len(self._signal_profile) == 0:
            self._signal_profile.append(_match_carla_traffic_light_state_to_cr(color))
        else:
            self._signal_profile[0] = _match_carla_traffic_light_state_to_cr(color)

    def add_color(self, color: carla.TrafficLightState):
        """Appends CARLA traffic light color to signal profile."""
        self._signal_profile.append(_match_carla_traffic_light_state_to_cr(color))

    def set_cr_light(self, cr_light: TrafficLight):
        """
        Setter for CommonRoad traffic light.

        :param cr_light: CommonRoad traffic light object.
        """
        self._cr_tl = cr_light

    def tick(self, time_step: int):
        """
        Update traffic light state for time step.

        :param time_step: Current time step.
        """
        new_color = self._cr_tl.get_state_at_time_step(time_step)
        self._actor.set_state(_match_cr_traffic_light_state_to_carla(new_color))

    @property
    def carla_actor(self):
        """
        Getter for CARLA traffic light object.

        :return: CARLA traffic light.
        """
        return self._actor

    @property
    def carla_position(self):
        """
        Getter for CARLA traffic light position.

        :return: CARLA traffic light position.
        """
        return self._carla_position

    def create_traffic_light_cycle(self) -> List[TrafficLightCycleElement]:
        """
        Creates a traffic light cycle for the given traffic light.

        :param traffic_light: the traffic light for the traffic light cycle is to be created
        :param starting_state: the state of the traffic light at the start
        """
        cycle = []
        current_state = self._signal_profile[0]
        duration = 0

        for state in self._signal_profile:
            if state == current_state:
                duration += 1
            else:
                cycle_element = TrafficLightCycleElement(current_state, duration)
                cycle.append(cycle_element)
                current_state = state
                duration = 1

        # Handle last cycle element
        cycle_element = TrafficLightCycleElement(current_state, duration)
        cycle.append(cycle_element)

        return cycle


def create_new_light(cr_light: TrafficLight, carla_lights: List[CarlaTrafficLight]) -> TrafficLight:

    best_carla_traffic_light = find_closest_traffic_light(carla_lights, cr_light)

    return TrafficLight(cr_light.traffic_light_id, best_carla_traffic_light.create_traffic_light_cycle(),
                        cr_light.position, time_offset=0, direction=cr_light.direction, active=True)


def find_closest_traffic_light(carla_lights: List[CarlaTrafficLight], cr_light: TrafficLight):
    best_carla_traffic_light = None
    best_diff = math.inf  # A big enough number
    cr_position = cr_light.position
    for light in carla_lights:
        diff_x = abs(light.carla_position[0] - cr_position[0])
        diff_y = abs(light.carla_position[1] + cr_position[1])  # We add since OPDR to CR Conversion inverses the sign
        cur_diff = math.sqrt(diff_x ** 2 + diff_y ** 2)

        if cur_diff < best_diff:
            best_diff = cur_diff
            best_carla_traffic_light = light
    return best_carla_traffic_light
