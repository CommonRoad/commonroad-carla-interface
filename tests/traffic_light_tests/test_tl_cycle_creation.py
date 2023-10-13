"""
Test cases for the creation of traffic light cycles from the signal profile.
"""
from tests.base_test_case.carla_server_test_case import CarlaServerTestCase
import random
from commonroad.scenario.traffic_light import TrafficLightState, TrafficLightCycleElement, TrafficLightCycle
from tests.utils import TCase


RANDOM_TEST_CASE_COUNT = 1000
MAX_TLC_LENGTH = 5
MAX_DURATION = 100

colors = [TrafficLightState.GREEN, TrafficLightState.RED, TrafficLightState.YELLOW, TrafficLightState.INACTIVE]


class TrafficLightCycleCreationTestCases(CarlaServerTestCase):
    """
    Test cases for the creation of traffic light cycles from the signal profile.
    """
    dummy_light = None

    def __init__(self, method_name: str = "runTest") -> None:
        super().__init__(method_name)

    @classmethod
    def setUpClass(cls):
        super().setUpClass()

        assert len(cls.carla_interface.traffic_lights) > 0
        cls.dummy_light = cls.carla_interface.traffic_lights[0]

        base_input = [[TrafficLightState.GREEN, TrafficLightState.YELLOW, TrafficLightState.RED]]
        base_expected = [TrafficLightCycle([
            TrafficLightCycleElement(TrafficLightState.GREEN, 1),
            TrafficLightCycleElement(TrafficLightState.YELLOW, 1),
            TrafficLightCycleElement(TrafficLightState.RED, 1)])]

        edge_input = [[], [TrafficLightState.RED]]
        edge_expected = [None, [TrafficLightCycle([TrafficLightCycleElement(TrafficLightState.RED, 1)])]]

        cls.basic_cases = []
        cls.edge_cases = []

        for b_input, b_expected, e_input, e_expected in zip(base_input, base_expected, edge_input, edge_expected):
            cls.dummy_light._signal_profile = b_input

            cls.basic_cases.append(TCase(expected=b_expected, output=cls.dummy_light.create_traffic_light_cycle()))

            cls.dummy_light._signal_profile = e_input

            cls.edge_cases.append(TCase(expected=e_expected, output=cls.dummy_light.create_traffic_light_cycle()))

    def test_random_cases(self):
        """
        Test random cases for traffic light cycle creation.
        """

        def test_create_traffic_light_cycle(carla_tl, rc_l, r_d, colors):
            cycle_elements = []
            signal_profile = []

            for j in range(rc_l):
                color = colors[j % len(colors)]

                cycle_elements.append(TrafficLightCycleElement(color, r_d))

                for _ in range(r_d):
                    signal_profile.append(color)
            cycle = TrafficLightCycle(cycle_elements)
            carla_tl._signal_profile = signal_profile
            created_cycle = carla_tl.create_traffic_light_cycle()

            return self.assertEqual(cycle, created_cycle)

        for _ in range(RANDOM_TEST_CASE_COUNT):
            random_cycle_length = random.randint(1, MAX_TLC_LENGTH)
            random_duration = random.randint(1, MAX_DURATION)
            random.shuffle(colors)
            test_create_traffic_light_cycle(self.dummy_light, random_cycle_length, random_duration, colors)
