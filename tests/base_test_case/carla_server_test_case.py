import os

from crcarla.carla_interface import CarlaInterface
from crcarla.helper.config import CarlaParams
from tests.base_test_case.my_test_case import StandardTestCase

"""
Todo when Using the CarlaServerTestCase class:

1. See StandardTestCase Classes Todo List
2. Overwrite the __init__ method to specify Carla parameters.
3. See the test_tl_cycle_creation.py file for reference on how to use the CarlaServerTestCase class.

"""


class CarlaServerTestCase(StandardTestCase):
    param = CarlaParams()
    carla_interface = None

    @classmethod
    def setUpClass(self):
        super().setUpClass()

        self.param.start_carla_server = False
        self.param.offscreen_mode = True

        self.carla_interface = CarlaInterface(self.param)

        config = os.path.join(self.log_dir, "config.json")
        self.param.save(config)

    def reset_server(self):
        self.ci = CarlaInterface(self.param)

    @classmethod
    def tearDownClass(cls):
        super().tearDownClass()
        cls.carla_interface.cleanup()
