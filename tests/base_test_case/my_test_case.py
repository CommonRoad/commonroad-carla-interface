import unittest
import inspect
import os
import shutil
import tests.utils
"""
Todo when Using the StandardTestCase class:

1. In the setUpClass method, append TestCase instances to the self.edge_cases and self.basic_cases lists.
2. Implement the test_random_cases method.
3. Use the assertWithLogging method instead of the regular assert method for logging of test failures.
4. See the test_tl_cycle_extraction.py file for reference on how to use the StandardTestCase class.

Also don't forget to call super().<method> when needed.
"""


class StandardTestCase(unittest.TestCase):
    basic_cases = None
    edge_cases = None
    log_dir = None

    @classmethod
    def setUpClass(cls):
        cls.basic_cases = None
        cls.edge_cases = None
        cls.log_dir = tests.utils.make_log_dir()

    def assertWithLogging(
        self, assertion, assertion_arguments, expected_response, actual_response
    ):
        try:
            assertion(*assertion_arguments)
        except AssertionError as ae:
            test_name = inspect.stack()[1][3]
            file_path = os.path.join(self.log_dir, "{}-Failure.log".format(test_name))
            with open(file_path, "w") as log_file:
                message = """
                    {} has failed
                    # Expected response:
                    # {}
                    # Actual response:
                    # {}
                    """.format(test_name, expected_response, actual_response)
                log_file.write(message)

            raise

    @classmethod
    def tearDownClass(cls):
        if len(os.listdir(cls.log_dir)) <= 1:
            shutil.rmtree(cls.log_dir)

    def test_edge_cases(self):
        """
        Test edge cases for traffic light cycle creation.
        """

        if self.edge_cases is None:
            raise NotImplementedError("Subclasses must specify the 'edge_cases' variable")

        for edge_case in self.edge_cases:
            self.assertWithLogging(self.assertEqual,
                                   [edge_case.expected, edge_case.output], edge_case.expected,
                                   edge_case.output)

    def test_basic_cases(self):
        """
        Test basic cases for traffic light cycle creation.
        """
        if self.basic_cases is None:
            raise NotImplementedError("Subclasses must specify the 'basic_cases' variable")

        for basic_case in self.basic_cases:
            self.assertEqual(basic_case.expected, basic_case.output)

    def test_random_cases(self):
        raise NotImplementedError("test_random_cases was not overwritten")
