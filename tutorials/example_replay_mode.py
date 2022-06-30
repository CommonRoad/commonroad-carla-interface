import time
import os

from carlacr.mode.carla_replay_mode import CarlaReplayMode

map_path = os.path.dirname(os.path.abspath("__file__")) + "/maps/"
scenario_path = os.path.dirname(os.path.abspath("__file__")) + "/scenarios/"
video_path = os.path.dirname(os.path.abspath("__file__")) + "/tutorials/video/"
name = "DEU_Test-1_1_T-1"

start_time = time.time()
replay = CarlaReplayMode(open_drive_map_path=map_path + name + ".xodr",
                         cr_scenario_path=scenario_path + name + ".xml")

replay.set_ego_vehicle_by_id(17)
replay.saving_video(create_video=True, video_path=video_path, video_as_mp4=True)

replay.visualize(sleep_time=5)
execution_time = (time.time() - start_time)

print('Execution time in seconds: ' + str(execution_time))
