"""
Test cases for the creation of traffic light cycles from the signal profile.
"""
import unittest
import random
from commonroad.scenario.traffic_sign import TrafficLightState, TrafficLightCycleElement
from tests.utils import EdgeCase
from carlacr.carla_interface import CarlaInterface

RANDOM_TEST_CASE_COUNT = 1000
MAX_TLC_LENGTH = 5
MAX_DURATION = 100

colors = [TrafficLightState.GREEN, TrafficLightState.RED,
          TrafficLightState.YELLOW, TrafficLightState.INACTIVE]


class TrafficLightCycleCreationTestCases(unittest.TestCase):
    """
    Test cases for the creation of traffic light cycles from the signal profile.
    """
    @classmethod
    def setUpClass(cls):
        cls.ci = CarlaInterface()
        assert len(cls.ci.traffic_lights) > 0
        cls.dummy_light = cls.ci.traffic_lights[0]

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_edge_cases(self):
        """
        Test edge cases for traffic light cycle creation.
        """
        edge_cases = [
            EdgeCase([], []),
            EdgeCase([TrafficLightState.RED], [
                     TrafficLightCycleElement(TrafficLightState.RED, 1)]),
            EdgeCase([TrafficLightState.GREEN, TrafficLightState.YELLOW, TrafficLightState.RED],
                     [TrafficLightCycleElement(TrafficLightState.GREEN, 1), TrafficLightCycleElement(TrafficLightState.YELLOW, 1), TrafficLightCycleElement(TrafficLightState.RED, 1)]),
        ]

        for edge_case in edge_cases:
            self.dummy_light._signal_profile = edge_case.case

            self.assertEqual(edge_case.expected,
                             self.dummy_light.create_traffic_light_cycle())

    def test_random_cases(self):
        """
        Test random cases for traffic light cycle creation.
        """
        def test_create_traffic_light_cycle(carla_tl, rc_l, r_d, colors):
            cycle = []
            signal_profile = []

            for j in range(rc_l):

                color = colors[j % len(colors)]

                cycle_element = TrafficLightCycleElement(color, r_d)
                cycle.append(cycle_element)

                for _ in range(r_d):
                    signal_profile.append(color)

            carla_tl._signal_profile = signal_profile
            created_cycle = carla_tl.create_traffic_light_cycle()
            return self.assertEqual(cycle, created_cycle)

        for _ in range(RANDOM_TEST_CASE_COUNT):
            random_cycle_length = random.randint(1, MAX_TLC_LENGTH)
            random_duration = random.randint(1, MAX_DURATION)
            random.shuffle(colors)
            test_create_traffic_light_cycle(
                self.dummy_light, random_cycle_length, random_duration, colors)
