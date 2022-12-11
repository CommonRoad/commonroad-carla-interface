import unittest
import os
import carla
import warnings

from carlacr.mode.carla_mode import CarlaMode
from commonroad.scenario.state import CustomState as State


class TestMode(unittest.TestCase):
    """Class for testing different modes."""

    def setUp(self) -> None:
        """Tests the setUp with a test map."""
        self.cwd = os.path.dirname(os.path.abspath(__file__))
        self.map_file = 'maps/DEU_Test-1_1_T-1.xodr'
        self.scenario_file = 'scenarios/DEU_Test-1_1_T-1.xml'
        self.carla_client = carla.Client("localhost", 2000)

    def test_without_mpl(self):
        """Tests Carla without a given or test map."""
        replay = CarlaMode(open_drive_map_path=self.cwd + "/../" + self.map_file,
                           cr_scenario_path=self.cwd + "/../" + self.scenario_file)

        with warnings.catch_warnings(record=True) as w:
            replay.visualize(sleep_time=5)

        self.assertEqual(len(w), 0,
                         msg="The following warnings were raised:\n" + "\n".join(str(w_tmp.message) for w_tmp in w) +
                             f"with scenario {self.scenario_file} and map {self.map_file}")

    def test_ego_vehicle(self):
        """Tests the ego vehicle."""
        replay = CarlaMode(open_drive_map_path=self.cwd + "/../" + self.map_file,
                           cr_scenario_path=self.cwd + "/../" + self.scenario_file)

        state = State(position=[35.1, 2.1], velocity=0, orientation=0.02, time_step=1)
        test_object = replay.create_dynamic_obstacles_ego(initial_state=state)

        self.assertIsNotNone(test_object)

    def test_with_ego_vehicle(self):
        """Tests Carla with an ego vehicle."""
        replay = CarlaMode(open_drive_map_path=self.cwd + "/../" + self.map_file,
                           cr_scenario_path=self.cwd + "/../" + self.scenario_file)

        state = State(position=[35.1, 2.1], velocity=0, orientation=0.02, time_step=1)
        test_object = replay.create_dynamic_obstacles_ego(initial_state=state)

        # GUI test
        replay.set_ego_vehicle(test_object)

        with warnings.catch_warnings(record=True) as w:
            replay.visualize(sleep_time=5)

        self.assertEqual(len(w), 0, msg="The following warnings were raised:\n" + "\n".join(
            str(w_tmp.message) for w_tmp in w) + f"with scenario {self.scenario_file} and map {self.map_file}")

        if not os.path.exists(self.cwd + '/video'):
            os.mkdir('video')

        replay.saving_video(create_video=True, video_path=self.cwd + '/video')

        # #TODO: try implement unittest for saving_video
        # try:
        #     replay.visualize(sleep_time=5)
        #     # self.carla_interface.saving_video(create_video=True, video_path=self.cwd + '/video')
        # except Exception as e:
        #     self.fail(f"replay mode failed with scenario {self.scenario_file}, map {self.map_file} and exception {e}")
        #
        # def test_saving_video(self):
        #     if not os.path.exists(self.cwd + '/video'):
        #         os.mkdir('video')
        #
        #     with warnings.catch_warnings(record=True) as w:
        #         self.carla_interface.saving_video(create_video=True, video_path=self.cwd + '/video')
        #
        #     self.assertEqual(len(w), 0, msg="The following warnings were raised:\n" + "\n"
        #     .join(str(w_tmp.message) for w_tmp in w))

    def test_traffic_generation(self, map_file: str = 'maps/DEU_Test-1_1_T-1.xodr'):
        """Tests the traffic generation with a given Map."""
        traffic_generation = CarlaMode(open_drive_map_path=self.cwd + "/../" + self.map_file,
                                       cr_scenario_path=self.cwd + "/../" + self.scenario_file)

        try:
            traffic_generation.traffic_generate(carla_vehicles=2, time_steps=40)
        except Exception as e:
            self.fail(f"traffic generation mode failed with scenario map {map_file} and exception {e}")

        # # should be a tighter bound on the later version
        # self.assertTrue(len(traffic_generation.scenario.dynamic_obstacles) >= 0)
        #
        # replay = traffic_generation.switch_to_replay_mode()
        # # GUI test
        # # replay.set_ego_vehicle(test_object)
        # try:
        #     replay.visualize(sleep_time=5)
        # except Exception as e:
        #     self.fail(f"traffic generation mode failed to create replay mode with exception {e}")


if __name__ == '__main__':
    unittest.main()
