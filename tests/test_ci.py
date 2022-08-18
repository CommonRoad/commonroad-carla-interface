# import unittest
# import os
# import carla
# import warnings
#
# from commonroad.scenario.trajectory import State
# from carlacr.mode.carla_replay_mode import CarlaReplayMode
#
#
# class TestReplayMode(unittest.TestCase):
#     def setUp(self) -> None:
#         self.cwd = os.path.dirname(os.path.abspath(__file__))
#         self.map_file = 'maps/DEU_Test-1_1_T-1.xodr'
#         self.scenario_file = 'scenarios/DEU_Test-1_1_T-1.xml'
#         self.carla_client = carla.Client("localhost", 2000)
#         # self.carla_interface = CarlaInterface(cr_scenario_file_path=self.cwd + "/../" + self.scenario_file,
#         #                                       open_drive_map_path=self.cwd + "/../" + self.map_file,
#         #                                       carla_client=self.carla_client)
#
#     def test_without_mpl(self):
#         replay = CarlaReplayMode(open_drive_map_path=self.cwd + "/../" + self.map_file,
#                                  cr_scenario_path=self.cwd + "/../" + self.scenario_file)
#
#         with warnings.catch_warnings(record=True) as w:
#             replay.visualize(sleep_time=5)
#
#         self.assertEqual(len(w), 0,
#                          msg="The following warnings were raised:\n" + "\n".join(str(w_tmp.message) for w_tmp in w) +
#                              f"with scenario {self.scenario_file} and map {self.map_file}")
#
#     def test_ego_vehicle(self):
#         replay = CarlaReplayMode(open_drive_map_path=self.cwd + "/../" + self.map_file,
#                                  cr_scenario_path=self.cwd + "/../" + self.scenario_file)
#
#         state = State(position=[35.1, 2.1], velocity=0, orientation=0.02, time_step=1)
#         test_object = replay.create_dynamic_obstacles_ego(initial_state=state)
#
#         self.assertIsNotNone(test_object)
#
#     def test_with_ego_vehicle(self):
#         replay = CarlaReplayMode(open_drive_map_path=self.cwd + "/../" + self.map_file,
#                                  cr_scenario_path=self.cwd + "/../" + self.scenario_file)
#
#         state = State(position=[35.1, 2.1], velocity=0, orientation=0.02, time_step=1)
#         test_object = replay.create_dynamic_obstacles_ego(initial_state=state)
#
#         # GUI test
#         replay.set_ego_vehicle(test_object)
#
#         with warnings.catch_warnings(record=True) as w:
#             replay.visualize(sleep_time=5)
#
#         self.assertEqual(len(w), 0, msg="The following warnings were raised:\n" + "\n".join(
#                 str(w_tmp.message) for w_tmp in w) + f"with scenario {self.scenario_file} and map {self.map_file}")
#
#         if not os.path.exists(self.cwd + '/video'):
#             os.mkdir('video')
#
#         replay.saving_video(create_video=True, video_path=self.cwd + '/video')
#
#         TODO: try implement unittest for saving_video
#         try:
#             replay.visualize(sleep_time=5)
#             # self.carla_interface.saving_video(create_video=True, video_path=self.cwd + '/video')
#         except Exception as e:
#             self.fail(f"replay mode failed with scenario {self.scenario_file}, map {self.map_file} and exception {e}")
#
#         def test_saving_video(self):
#             if not os.path.exists(self.cwd + '/video'):
#                 os.mkdir('video')
#
#             with warnings.catch_warnings(record=True) as w:
#                 self.carla_interface.saving_video(create_video=True, video_path=self.cwd + '/video')
#
#             self.assertEqual(len(w), 0, msg="The following warnings were raised:\n" + "\n"
#             .join(str(w_tmp.message) for w_tmp in w))
#
#

# if __name__ == '__main__':
#     unittest.main()
