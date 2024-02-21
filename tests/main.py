"""Load all tests and run them."""

import os
import sys
import unittest

from tests.traffic_light_tests.test_tl_cycle_creation import (
    TrafficLightCycleCreationTestCases,
)

# from tests.traffic_light_tests.test_tl_cycle_extraction import TrafficLightCycleExtractionTestCases


os.environ["SDL_VIDEODRIVER"] = "dummy"

test_loader = unittest.TestLoader()

tlcc_tests = test_loader.loadTestsFromTestCase(TrafficLightCycleCreationTestCases)
# tlce_tests = test_loader.loadTestsFromTestCase(TrafficLightCycleExtractionTestCases)

test_suite = unittest.TestSuite()
test_suite.addTests([tlcc_tests])  # , tlce_tests])

result = unittest.TextTestRunner(verbosity=2).run(test_suite)
sys.exit(not result.wasSuccessful())
