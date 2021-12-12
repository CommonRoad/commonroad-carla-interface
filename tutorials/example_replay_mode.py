import time

from commonroad.scenario.trajectory import State

from carlacr.carla_replay_mode import CarlaReplayMode

map_path = "../maps/"
scenario_path = "../scenarios/"

name = "DEU_Test-1_1_T-1"
startTime = time.time()
replay = CarlaReplayMode(open_drive_map_path=map_path + name + ".xodr", cr_scenario_path=scenario_path + name + ".xml")

replay.set_ego_vehicle(replay.obstacle_by_id(6))

# pass obstacle in commonroad for viewing
# state = State(position=[10.0, 1.5],
#               velocity=0,
#               orientation=0.02,
#               time_step=1)
# replay.set_ego_vehicle(replay.create_dynamic_obstacles_ego(initial_state=state))

replay.visualize(sleep_time=5, saving_video=False, video_path="video")
executionTime = (time.time() - startTime)

print('Execution time in seconds: ' + str(executionTime))
