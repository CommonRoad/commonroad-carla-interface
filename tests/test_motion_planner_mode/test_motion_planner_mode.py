import unittest

from commonroad.scenario.scenario import Scenario
from carlacr.interface.carla_interface import MotionPlanner


class TestMotionPlannerMode(unittest.TestCase):
    def test_motion_planner_mode(self, open_drive_map_path: str,
                                 cr_scenario_path: str = None,
                                 cr_scenario: Scenario = None,
                                 motion_planner: MotionPlanner = None,
                                 vehicle_id: int = -1):
        pass


if __name__ == '__main__':
    unittest.main()
