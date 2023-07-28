"""
Test cases for the traffic light cycle extraction function.
"""
import unittest
import random
from commonroad.scenario.traffic_sign import TrafficLightState, TrafficLightCycleElement
from carlacr.objects.traffic_light import extract_cycle_from_history

RANDOM_TEST_CASE_COUNT = 10000
TL_CYCLE_MAX_LENGTH = 10
MAX_DURATION = 100


class TrafficLightTestCases(unittest.TestCase):
    """
    Test cases for the traffic light cycle extraction function.
    """

    def test_basic_3_element_extraction(self):
        """
        Test basic extraction of a 3 element cycle.
        """
        history = [
            TrafficLightCycleElement(
                state=TrafficLightState.RED, duration=300),
            TrafficLightCycleElement(
                state=TrafficLightState.GREEN, duration=100),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=31),
            TrafficLightCycleElement(
                state=TrafficLightState.RED, duration=322),
            TrafficLightCycleElement(
                state=TrafficLightState.GREEN, duration=100),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=31),
        ]

        expected_cycle = [
            TrafficLightCycleElement(
                state=TrafficLightState.RED, duration=322),
            TrafficLightCycleElement(
                state=TrafficLightState.GREEN, duration=100),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=31),
        ]
        self.assertEqual(extract_cycle_from_history(history), expected_cycle)

    def test_basic_4_element_extraction(self):
        """
        Test basic extraction of a 4 element cycle.
        """
        history = [
            TrafficLightCycleElement(
                state=TrafficLightState.RED, duration=300),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=20),
            TrafficLightCycleElement(
                state=TrafficLightState.GREEN, duration=100),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=31),
            TrafficLightCycleElement(
                state=TrafficLightState.RED, duration=322),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=20),
            TrafficLightCycleElement(
                state=TrafficLightState.GREEN, duration=100),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=31),
        ]
        expected_cycle = [
            TrafficLightCycleElement(
                state=TrafficLightState.RED, duration=322),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=20),
            TrafficLightCycleElement(
                state=TrafficLightState.GREEN, duration=100),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=31),
        ]

        self.assertEqual(extract_cycle_from_history(history), expected_cycle)

    def test_warning_extract_cycle_from_history(self):
        """
        Test warning extraction of a 4 element cycle.
        """
        history = [
            TrafficLightCycleElement(
                state=TrafficLightState.RED, duration=300),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=100),
            TrafficLightCycleElement(
                state=TrafficLightState.GREEN, duration=100),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=100),
        ]

        self.assertEqual(extract_cycle_from_history(history), history)

    def test_edge_cases_extract_cycle_from_history(self):
        """
        Test edge cases extraction of a 4 element cycle.
        """
        history = [
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=300),
        ]
        self.assertEqual(extract_cycle_from_history(history), history)

        # should throw warning
        history = [
            TrafficLightCycleElement(
                state=TrafficLightState.RED, duration=300),
            TrafficLightCycleElement(
                state=TrafficLightState.YELLOW, duration=300),
            TrafficLightCycleElement(
                state=TrafficLightState.GREEN, duration=300),
        ]
        self.assertEqual(extract_cycle_from_history(history), history)

        history = [
            TrafficLightCycleElement(state=TrafficLightState.RED, duration=10),
            TrafficLightCycleElement(
                state=TrafficLightState.GREEN, duration=300),
            TrafficLightCycleElement(
                state=TrafficLightState.RED, duration=300),
            TrafficLightCycleElement(
                state=TrafficLightState.GREEN, duration=20),
        ]
        expected_cycle = [TrafficLightCycleElement(state=TrafficLightState.RED, duration=300),
                          TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=300),]

        self.assertEqual(extract_cycle_from_history(history), expected_cycle)

    def test_random_3element_cycle_extraction(self):
        """
        Test random extraction of a 3 element cycle.
        """
        original_cycle_colors = [TrafficLightState.RED,
                                 TrafficLightState.GREEN, TrafficLightState.YELLOW]
        original_cycle_durations = [random.randint(
            1, MAX_DURATION) for _ in range(3)]

        for _ in range(RANDOM_TEST_CASE_COUNT):

            random.shuffle(original_cycle_colors)
            random.shuffle(original_cycle_durations)

            length = random.randint(5, TL_CYCLE_MAX_LENGTH)

            history = []
            for i in range(length):
                history.append(TrafficLightCycleElement(
                    state=original_cycle_colors[i % 3], duration=original_cycle_durations[i % 3]))

            history[0].duration = random.randint(1, MAX_DURATION)
            history[-1].duration = random.randint(1, MAX_DURATION)

            expected_cycle = [TrafficLightCycleElement(state=color, duration=duration)
                              for color, duration in
                              zip(original_cycle_colors, original_cycle_durations)]

            self.assertEqual(extract_cycle_from_history(
                history), expected_cycle)

    def test_random_4element_cycle_extraction(self):
        """
        Terst random extraction of a 4 element cycle.
        """
        original_cycle_colors = [TrafficLightState.RED, TrafficLightState.YELLOW,
                                 TrafficLightState.GREEN, TrafficLightState.YELLOW]
        original_cycle_durations = [random.randint(
            1, MAX_DURATION) for _ in range(4)]

        for _ in range(RANDOM_TEST_CASE_COUNT):
            start = random.randint(0, 3)
            history = []

            for i in range(start, random.randint(6 + start, TL_CYCLE_MAX_LENGTH)):
                history.append(TrafficLightCycleElement(
                    state=original_cycle_colors[i % 4], duration=original_cycle_durations[i % 4]))

            history[0].duration = random.randint(1, MAX_DURATION)
            history[-1].duration = random.randint(1, MAX_DURATION)

            expected_cycle = []
            for i in range(4):
                expected_cycle.append(TrafficLightCycleElement(state=original_cycle_colors[(
                    start + i) % 4], duration=original_cycle_durations[(start + i) % 4]))

            self.assertEqual(extract_cycle_from_history(
                history), expected_cycle)