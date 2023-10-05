import logging
import numpy as np
from typing import Optional, List, Tuple
import math

import carla

from commonroad.scenario.traffic_light import (TrafficLight, TrafficLightState, TrafficLightCycleElement,
                                               TrafficLightCycle)

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def _match_carla_traffic_light_state_to_cr(carla_state: carla.TrafficLightState) -> TrafficLightState:
    """
    Match CARLA traffic light state to CommonRoad traffic light state

    :param carla_state: CARLA traffic light state.
    :return: CommonRoad traffic light state
    """
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
    """
    Match CommonRoad traffic light state to CARLA traffic light state

    :param cr_state: CommonRoad traffic light state.
    :return: CARLA traffic light state
    """
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
        """
        Appends CARLA traffic light color to signal profile.

        :param color: CARLA traffic light state color.
        """
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
        if self._cr_tl is not None:
            new_color = self._cr_tl.get_state_at_time_step(time_step)
            self._actor.set_state(_match_cr_traffic_light_state_to_carla(new_color))
            logger.debug("CarlaTrafficLight::tick: Successfully called tick!")
        else:
            logger.warning("CarlaTrafficLight::tick: Tick called on traffic light where _cr_tl is None!.")

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

    def create_traffic_light_cycle(self) -> Optional[TrafficLightCycle]:
        """
        Creates a CommonRoad traffic light cycle for traffic light.

        :return: List of CommonRoad traffic light cycle elements.
        """
        if len(self._signal_profile) == 0:
            return None

        if len(self._signal_profile) == 0:
            return None

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

        return TrafficLightCycle(cycle)


def create_new_light(cr_light: TrafficLight, carla_lights: List[CarlaTrafficLight]) -> TrafficLight:
    """
    Creates traffic light interface given CommonRoad traffic light and all available CARLA traffic light.
    Uses the closest CARLA traffic light.

    :param cr_light: CommonRoad traffic light.
    :param carla_lights: List of CARLA traffic lights.
    :return: Traffic light interface.
    """
    best_carla_traffic_light = find_closest_traffic_light(carla_lights, cr_light.position)

    return TrafficLight(cr_light.traffic_light_id, cr_light.position,
                        best_carla_traffic_light.create_traffic_light_cycle(), direction=cr_light.direction,
                        active=True)


def find_closest_traffic_light(carla_lights: List[CarlaTrafficLight], position: np.array):
    """
    Extracts the closes CARLA traffic light for a given position using Euclidean distance.

    :param carla_lights: List of CARLA traffic lights.
    :param position: Position for which closes CARLA traffic lights should be extracted.
    """
    best_carla_traffic_light = None
    best_diff = math.inf

    for light in carla_lights:
        diff_x = abs(light.carla_position[0] - position[0])
        # We add since map is mirrored compared to CommonRoad
        diff_y = abs(light.carla_position[1] + position[1])
        cur_diff = math.sqrt(diff_x ** 2 + diff_y ** 2)

        if cur_diff < best_diff:
            best_diff = cur_diff
            best_carla_traffic_light = light

    return best_carla_traffic_light


def get_tls_values(tls: List[TrafficLightState]) -> Tuple[int, int, int, int]:
    """
    Extracts the first full red, first full green, yellow after red and
    yellow after green index from a list of traffic light states.

    :param tls: List of traffic light states.
    :return: Tuple of indexes for the first none partial encounter of RED, GREEN,
    YELLOW (after RED and GREEN) traffic light states.
    """
    first_full_red = None
    first_full_green = None
    yellow_after_red = None
    yellow_after_green = None

    for i, t_l in enumerate(tls):
        if i in (0, len(tls) - 1):
            continue

        if t_l.state == TrafficLightState.RED and first_full_red is None:
            first_full_red = i
        elif t_l.state == TrafficLightState.GREEN and first_full_green is None:
            first_full_green = i
        elif (t_l.state == TrafficLightState.YELLOW and
              tls[i - 1].state == TrafficLightState.RED and
              yellow_after_red is None):
            yellow_after_red = i
        elif (t_l.state == TrafficLightState.YELLOW and
              tls[i - 1].state == TrafficLightState.GREEN and
              yellow_after_green is None):
            yellow_after_green = i

        if (first_full_red is not None and first_full_green is not None and
                yellow_after_red is not None and yellow_after_green is not None):
            break

    return first_full_red, first_full_green, yellow_after_red, yellow_after_green


def get_cycle_duration(tls: List[TrafficLight], cycle, first_full_red: int,
                       first_full_green: int, yellow_after_red: int,
                       yellow_after_green: int, tl_state: TrafficLightState) -> int:
    """
    Get the duration of the cycle.

    :param tls: List of TrafficLight objects.
    :param cycle: List of TrafficLightCycleElement objects.
    :param first_full_red: Index of first full red traffic light state.
    :param first_full_green: Index of first full green traffic light state.
    :param yellow_after_red: Index of yellow traffic light state after red.
    :param yellow_after_green: Index of yellow traffic light state after green.
    :param tl_state: Traffic light state.
    :return: Duration of the cycle.
    """
    if tl_state == TrafficLightState.RED:
        return tls[first_full_red].duration
    if tl_state == TrafficLightState.GREEN:
        return tls[first_full_green].duration
    if cycle[-1].state == TrafficLightState.RED:
        return tls[yellow_after_red].duration
    if cycle[-1].state == TrafficLightState.GREEN:
        return tls[yellow_after_green].duration
    return 0


def rotate_cycle(cycle: List[TrafficLightCycleElement],
                 tls: List[TrafficLight]) -> List[TrafficLightCycleElement]:
    """
    Rotates the extracted cycle to match the input list.

    :param cycle: List of TrafficLight objects.
    :param tls: List of TrafficLight objects gathered from Carla state.
    :return: Rotated cycle, where the first element matches the first element of tls.
    """
    for _ in range(len(cycle)):
        if cycle[0].state == tls[0].state and cycle[1].state == tls[1].state:
            break
        cycle = cycle[1:] + [cycle[0]]
    else:
        raise ValueError(
            "Could not rotate the extracted cycle to match the input list")
    return cycle


def extract_cycle_from_history(tls: List[TrafficLight]) -> List[TrafficLightCycleElement]:
    """
    Extracts the current traffic light cycle from a list of traffic lights.
    Requires tls to contain a full cycle.

    :param tls: List of TrafficLight objects gathered from state Carla.
    :return: List of TrafficLightCycleElement objects. The extracted 2(RED,GREEN,RED..)
    ,3(yellow between RED and GREEN or GREEN and RED)
    or 4 element cycle(YELLOW between RED and GREEN AND GREEN and RED).
    """
    # Find the first none partial RED or GREEN element in the list
    if len(tls) < 3:
        logger.info(
            "To few elements in input list! Extracted cycle might be incorrect!")
        return tls

    first_full_red, first_full_green, yellow_after_red, yellow_after_green = get_tls_values(tls)

    if first_full_red is None or first_full_green is None:
        logger.info(
            """Could not find a full RED or GREEN state in input list!
            "Extracted cycle might be incorrect!""")
        return tls

    if yellow_after_red is None and yellow_after_green is None and any(
            element.state == TrafficLightState.YELLOW for element in tls):
        logger.info(
            """Could not find a full RED or GREEN state
            in input list! Extracted cycle might be incorrect!""")
        return tls

    cycle = []
    found_end = False
    cycle_start = (0 if tls[0].state in (TrafficLightState.RED, TrafficLightState.GREEN) else min(
        first_full_green, first_full_red))

    cycle.append(TrafficLightCycleElement(
        tls[cycle_start].state, get_cycle_duration(tls, cycle, first_full_red,
                                                   first_full_green, yellow_after_red,
                                                   yellow_after_green, tls[cycle_start].state)))

    for i in range(cycle_start + 1, len(tls)):
        cycle.append(TrafficLightCycleElement(
            tls[i].state,
            get_cycle_duration(tls, cycle, first_full_red, first_full_green, yellow_after_red,
                               yellow_after_green,
                               tls[i].state)))
        if tls[i].state == tls[cycle_start].state:
            found_end = True
            break
    if not found_end:
        for i in range(cycle_start + 1):
            cycle.append(TrafficLightCycleElement(
                tls[i].state,
                get_cycle_duration(tls, cycle, first_full_red, first_full_green,
                                   yellow_after_red, yellow_after_green, tls[i].state)))
            if tls[i].state == tls[cycle_start].state:
                found_end = True
                break

    cycle.pop()

    # Rotate the cycle so that it starts at the same state as the input list
    cycle = rotate_cycle(cycle, tls)

    return cycle
