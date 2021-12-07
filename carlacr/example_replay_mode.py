import time
from carlacr.carla_replay_mode import CarlaReplayMode

map_path = "../maps/"
scenario_path = "../scenarios/"

name = "DEU_Test-1_1_T-1"
startTime = time.time()
replay = CarlaReplayMode(scenario_path + name + ".xml", map_path + name + ".xodr")

replay.set_ego_vehicle(replay.find_car_with_CR_ID(6))
# replay.set_ego_vehicle(replay.find_car_with_CR_ID(7))
# replay.set_ego_vehicle(replay.create_static_obstacles_ego([10, 1.5], 0.0))
replay.visualize(sleep_time=5, saving_video=False, video_path="/home/")
executionTime = (time.time() - startTime)

print('Execution time in seconds: ' + str(executionTime))
