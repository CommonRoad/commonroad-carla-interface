import logging
import numpy as np
from typing import Optional, List
import math

import carla

from commonroad.scenario.traffic_sign import TrafficLight, TrafficLightState, TrafficLightCycleElement

logger = logging.getLogger(__name__)


# def create_carla_transform(position: np.array, orientation: float, z_position: float = 2.0):
#     transform = carla.Transform(
#             carla.Location(x=position[0], y=-position[1], z=z_position),
#             carla.Rotation(yaw=(-(180 * orientation) / np.pi)))
#     return transform




#
# def _get_cycle_of_matching_traffic_light(self, cr_traffic_light: TrafficLight):
#     """
#     Finds the matching lanelet for the CARLA traffic light object.
#
#     :param carla_traffic_light: the CARLA traffic light object for which a matching lanelet is to be found
#     """
#     best_carla_traffic_light_id = None
#     # A big enough number
#     best_diff = 99999
#     cr_position = cr_traffic_light.position
#
#     for actor in self.world.get_actors():
#         if "light" in actor.type_id:
#             carla_location = actor.get_location()
#             diff_x = abs(carla_location.x - cr_position[0])
#             diff_y = abs(carla_location.y + cr_position[1])  # We add since OPDR to CR Conversion inverses the sign
#
#             cur_diff = sqrt(diff_x**2 + diff_y**2)
#
#             if cur_diff < best_diff:
#                 best_diff = cur_diff
#                 best_carla_traffic_light_id = actor.id
#
#     return self._create_traffic_light_cycle(best_carla_traffic_light_id)
#
# def _change_cycle_of_traffic_lights(self):
#     """Change the cycle of the CR traffic lights in the scenario to the cycle of the nearest CARLA traffic light."""
#
#     lanelet_network = self.scenario.lanelet_network
#     lanelets = lanelet_network.lanelets
#
#     for lanelet in lanelets:
#         for traffic_light_id in lanelet.traffic_lights:
#             traffic_light = lanelet_network.find_traffic_light_by_id(traffic_light_id)
#             traffic_light.cycle = self._get_cycle_of_matching_traffic_light(traffic_light)

class CarlaTrafficLight:
    def __init__(self, carla_id: int, position: carla.Location, cr_tl: Optional[TrafficLight] = None):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        self._carla_id = carla_id
        self._carla_position = np.array([position.x, position.y, position.z])
        self._signal_profile: List[TrafficLightState] = []
        self._cr_tl = cr_tl

    def set_initial_color(self, color: carla.TrafficLightState):
        if len(self._signal_profile) == 0:
            self._signal_profile.append(self._match_traffic_light_state(color))
        else:
            self._signal_profile[0] = self._match_traffic_light_state(color)

    def add_color(self, color: carla.TrafficLightState):
        self._signal_profile.append(self._match_traffic_light_state(color))

    @property
    def carla_id(self):
        return self._carla_id

    @property
    def carla_position(self):
        return self._carla_position

    def _match_traffic_light_state(self, carla_state: carla.TrafficLightState) -> TrafficLightState:
        if carla_state == carla.TrafficLightState.Green:
            return TrafficLightState.GREEN
        elif carla_state == carla.TrafficLightState.Red:
            return TrafficLightState.RED
        elif carla_state == carla.TrafficLightState.Yellow:
            return TrafficLightState.YELLOW
        elif carla_state == carla.TrafficLightState.Off:
            return TrafficLightState.INACTIVE
        else:
            return TrafficLightState.RED

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

    best_carla_traffic_light = None
    best_diff = math.inf # A big enough number
    cr_position = cr_light.position

    for light in carla_lights:
        diff_x = abs(light.carla_position[0] - cr_position[0])
        diff_y = abs(light.carla_position[1] + cr_position[1])  # We add since OPDR to CR Conversion inverses the sign
        cur_diff = math.sqrt(diff_x ** 2 + diff_y ** 2)

        if cur_diff < best_diff:
            best_diff = cur_diff
            best_carla_traffic_light = light

    return TrafficLight(cr_light.traffic_light_id, best_carla_traffic_light.create_traffic_light_cycle(),
                        cr_light.position, time_offset=0, direction=cr_light.direction, active=True)