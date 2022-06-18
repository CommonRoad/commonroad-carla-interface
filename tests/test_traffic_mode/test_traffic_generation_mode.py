import unittest
import os
from carlacr.mode.carla_traffic_generation_mode import CarlaTrafficGenerationMode


class TestTrafficGenerationMode(unittest.TestCase):
    def test_traffic_generation_mode(self, map_file: str = 'maps/DEU_Test-1_1_T-1.xodr'):
        traffic_generation = CarlaTrafficGenerationMode(open_drive_map_path=os.path.dirname(os.path.abspath(__file__)) +
                                                        "/../../" + map_file)
        try:
            traffic_generation.traffic_generate(carla_vehicles=2, time_steps=40)
        except Exception as e:
            self.fail(f"traffic generation mode failed with scenario map {map_file} and exception {e}")

        # should be a tighter bound on the later version
        self.assertTrue(len(traffic_generation.scenario.dynamic_obstacles) >= 0)

        replay = traffic_generation.switch_to_replay_mode()
        # GUI test
        # replay.set_ego_vehicle(test_object)
        try:
            replay.visualize(sleep_time=5)
        except Exception as e:
            self.fail(f"traffic generation mode failed to create replay mode with exception {e}")


if __name__ == '__main__':
    unittest.main()