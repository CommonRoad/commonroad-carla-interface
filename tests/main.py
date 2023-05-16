"""Load all tests and run them."""
import unittest
import traffic_light_tests.test_tl_cycle_creation as tl_test
import traffic_light_tests.test_tl_cycle_extraction as tl_test2


tl_suite = unittest.TestLoader().loadTestsFromModule(tl_test2)
unittest.TextTestRunner(verbosity=2).run(tl_suite)
