import unittest
import os

from commonroad.scenario.trajectory import State

from carlacr.mode.carla_replay_mode import CarlaReplayMode


class TestReplayMode(unittest.TestCase):
    def test_replay_mode(self, scenario_file: str = 'scenarios/DEU_Test-1_1_T-1.xml',
                         map_file: str = 'maps/DEU_Test-1_1_T-1.xodr'):
        replay = CarlaReplayMode(open_drive_map_path=os.path.dirname(os.path.abspath(__file__)) + "/../../" + map_file,
                                 cr_scenario_path=os.path.dirname(os.path.abspath(__file__)) + "/../../" + scenario_file)

        state = State(position=[35.1, 2.1], velocity=0, orientation=0.02, time_step=1)
        try:
            test_object = replay.create_dynamic_obstacles_ego(initial_state=state)
            self.assertIsNotNone(test_object)
        except Exception as e:
            self.fail(f"replay mode failed to create object with scenario {scenario_file}, map {map_file} and exception {e}")

        # GUI test
        replay.set_ego_vehicle(test_object)
        try:
            replay.visualize(sleep_time=5)
        except Exception as e:
            self.fail(f"replay mode failed with scenario {scenario_file}, map {map_file} and exception {e}")


if __name__ == '__main__':
    unittest.main()

