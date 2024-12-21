# """
# Test cases for the traffic light cycle extraction function.
# """
#
# import random
# from tests.base_test_case.my_test_case import StandardTestCase
# from tests.utils import TCase
# from commonroad.scenario.traffic_light import TrafficLightState, TrafficLightCycleElement
# # from crcarla.objects.traffic_light import extract_cycle_from_history
#
#
# RANDOM_TEST_CASE_COUNT = 10000
# TL_CYCLE_MAX_LENGTH = 10
# MAX_DURATION = 100
#
#
# class TrafficLightCycleExtractionTestCases(StandardTestCase):
#     """
#     Test cases for the traffic light cycle extraction function.
#     """
#
#     @classmethod
#     def setUpClass(cls):
#         super().setUpClass()
#
#         basic_input_3_element = [
#             [
#                 TrafficLightCycleElement(state=TrafficLightState.RED, duration=300),
#                 TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=100),
#                 TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=31),
#                 TrafficLightCycleElement(state=TrafficLightState.RED, duration=322),
#                 TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=100),
#                 TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=31),
#             ]
#         ]
#         basic_expected_3_element = [
#             TrafficLightCycleElement(state=TrafficLightState.RED, duration=322),
#             TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=100),
#             TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=31),
#         ]
#         basic_input_4_element = [
#             [
#                 TrafficLightCycleElement(state=TrafficLightState.RED, duration=300),
#                 TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=20),
#                 TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=100),
#                 TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=31),
#                 TrafficLightCycleElement(state=TrafficLightState.RED, duration=322),
#                 TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=20),
#                 TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=100),
#                 TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=31),
#             ]
#         ]
#         basic_expected_4_element = [
#             TrafficLightCycleElement(state=TrafficLightState.RED, duration=322),
#             TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=20),
#             TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=100),
#             TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=31),
#         ]
#
#         cls.basic_cases = [
#             TCase(
#                 input=basic_input_3_element,
#                 function=extract_cycle_from_history,
#                 expected=basic_expected_3_element,
#             ),
#             TCase(
#                 input=basic_input_4_element,
#                 function=extract_cycle_from_history,
#                 expected=basic_expected_4_element,
#             ),
#         ]
#
#         edge_input_1 = [
#             [
#                 TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=300),
#             ]
#         ]
#
#         edge_expected_1 = edge_input_1[0]
#
#         edge_input_2 = [
#             [
#                 TrafficLightCycleElement(state=TrafficLightState.RED, duration=300),
#                 TrafficLightCycleElement(state=TrafficLightState.YELLOW, duration=300),
#                 TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=300),
#             ]
#         ]
#
#         edge_expected_2 = edge_input_2[0]
#
#         edge_input_3 = [
#             [
#                 TrafficLightCycleElement(state=TrafficLightState.RED, duration=10),
#                 TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=300),
#                 TrafficLightCycleElement(state=TrafficLightState.RED, duration=300),
#                 TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=20),
#             ]
#         ]
#         edge_expected_3 = [
#             TrafficLightCycleElement(state=TrafficLightState.RED, duration=300),
#             TrafficLightCycleElement(state=TrafficLightState.GREEN, duration=300),
#         ]
#
#         cls.edge_cases = [
#             TCase(
#                 input=edge_input_1,
#                 function=extract_cycle_from_history,
#                 expected=edge_expected_1,
#             ),
#             TCase(
#                 input=edge_input_2,
#                 function=extract_cycle_from_history,
#                 expected=edge_expected_2,
#             ),
#             TCase(
#                 input=edge_input_3,
#                 function=extract_cycle_from_history,
#                 expected=edge_expected_3,
#             ),
#         ]
#
#     def test_random_cases(self):
#         self._test_random_3element_cycle_extraction()
#         self._test_random_4element_cycle_extraction()
#
#     def _test_random_3element_cycle_extraction(self):
#         """
#         Test random extraction of a 3 element cycle.
#         """
#         original_cycle_colors = [
#             TrafficLightState.RED,
#             TrafficLightState.GREEN,
#             TrafficLightState.YELLOW,
#         ]
#         original_cycle_durations = [random.randint(1, MAX_DURATION) for _ in range(3)]
#
#         for _ in range(RANDOM_TEST_CASE_COUNT):
#             random.shuffle(original_cycle_colors)
#             random.shuffle(original_cycle_durations)
#
#             length = random.randint(5, TL_CYCLE_MAX_LENGTH)
#
#             history = []
#             for i in range(length):
#                 history.append(
#                     TrafficLightCycleElement(
#                         state=original_cycle_colors[i % 3],
#                         duration=original_cycle_durations[i % 3],
#                     )
#                 )
#
#             history[0].duration = random.randint(1, MAX_DURATION)
#             history[-1].duration = random.randint(1, MAX_DURATION)
#
#             expected_cycle = [
#                 TrafficLightCycleElement(state=color, duration=duration)
#                 for color, duration in zip(
#                     original_cycle_colors, original_cycle_durations
#                 )
#             ]
#
#             self.assertEqual(extract_cycle_from_history(history), expected_cycle)
#
#     def _test_random_4element_cycle_extraction(self):
#         """
#         Test random extraction of a 4 element cycle.
#         """
#         original_cycle_colors = [
#             TrafficLightState.RED,
#             TrafficLightState.YELLOW,
#             TrafficLightState.GREEN,
#             TrafficLightState.YELLOW,
#         ]
#         original_cycle_durations = [random.randint(1, MAX_DURATION) for _ in range(4)]
#
#         for _ in range(RANDOM_TEST_CASE_COUNT):
#             start = random.randint(0, 3)
#             history = []
#
#             for i in range(start, random.randint(6 + start, TL_CYCLE_MAX_LENGTH)):
#                 history.append(
#                     TrafficLightCycleElement(
#                         state=original_cycle_colors[i % 4],
#                         duration=original_cycle_durations[i % 4],
#                     )
#                 )
#
#             history[0].duration = random.randint(1, MAX_DURATION)
#             history[-1].duration = random.randint(1, MAX_DURATION)
#
#             expected_cycle = []
#             for i in range(4):
#                 expected_cycle.append(
#                     TrafficLightCycleElement(
#                         state=original_cycle_colors[(start + i) % 4],
#                         duration=original_cycle_durations[(start + i) % 4],
#                     )
#                 )
#
#             self.assertEqual(extract_cycle_from_history(history), expected_cycle)
