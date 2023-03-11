import logging
import numpy as np
from typing import Optional

import carla

from commonroad.scenario.traffic_sign import TrafficLight

logger = logging.getLogger(__name__)


def create_carla_transform(position: np.array, orientation: float, z_position: float = 2.0):
    transform = carla.Transform(
            carla.Location(x=position[0], y=-position[1], z=z_position),
            carla.Rotation(yaw=(-(180 * orientation) / np.pi)))
    return transform


class CarlaTrafficLight:
    def __init__(self, cr_tl: TrafficLight, config):
        """
        Initializer of the obstacle.

        :param cr_obstacle: the underlying CommonRoad obstacle
        """
        self._carla_id = None
        self._config = config
        self._world = None
        self._trajectory = []  # TODO delete later and use cr-io history
        if cr_tl is not None:
            self._commonroad_id = cr_tl.traffic_light_id
            self._cr_base = cr_tl
        else:
            self._commonroad_id = None
            self._cr_base: Optional[TrafficLight] = None

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
    # def _create_traffic_light_cycle_element(self, state: carla.TrafficLightState, duration: int):
    #     """
    #     Creates a CommonRoad TrafficLightCycleElement for the given CARLA TrafficLightState and duration.
    #
    #     :param state: the CARLA traffic light state
    #     :param duration: the duration of the state
    #     """
    #     cycle_element = None
    #     if state == carla.TrafficLightState.Green:
    #         cycle_element = TrafficLightCycleElement(TrafficLightState.GREEN, duration)
    #     elif state == carla.TrafficLightState.Red:
    #         cycle_element = TrafficLightCycleElement(TrafficLightState.RED, duration)
    #     elif state == carla.TrafficLightState.Yellow:
    #         cycle_element = TrafficLightCycleElement(TrafficLightState.YELLOW, duration)
    #     return cycle_element
    #
    # def _create_traffic_light_cycle(self, traffic_light_id: int):
    #     """
    #     Creates a traffic light cycle for the given traffic light.
    #
    #     :param traffic_light: the traffic light for the traffic light cycle is to be created
    #     :param starting_state: the state of the traffic light at the start
    #     """
    #     state_list = self.traffic_lights[traffic_light_id]
    #     cycle = []
    #     current_state = state_list[0]
    #     duration = 0
    #
    #     for state in state_list:
    #         if state == current_state:
    #             duration += 1
    #         else:
    #             cycle_element = self._create_traffic_light_cycle_element(current_state, duration)
    #             cycle.append(cycle_element)
    #             current_state = state
    #             duration = 1
    #
    #     # Handle last cycle element
    #     cycle_element = self._create_traffic_light_cycle_element(current_state, duration)
    #     cycle.append(cycle_element)
    #
    #     return cycle
