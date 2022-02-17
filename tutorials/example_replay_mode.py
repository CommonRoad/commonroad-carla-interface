import time

from carlacr.mode.carla_replay_mode import CarlaReplayMode

map_path = "../maps/"
scenario_path = "../scenarios/"

name = "DEU_Test-1_1_T-1"
startTime = time.time()
replay = CarlaReplayMode(open_drive_map_path= "/home/hoaquin/Desktop/test.xodr", cr_scenario_path="/home/hoaquin/Desktop/test_result.xml")

replay.set_ego_vehicle_by_id(17)
replay.saving_video(create_video=True,video_path="/home/hoaquin/Desktop",video_asMP4=True)
# # pass obstacle in commonroad for viewing
# state = State(position=[35.1, 2.1],
#               velocity=0,
#               orientation=0.02,
#               time_step=1)
# replay.set_ego_vehicle(replay.create_dynamic_obstacles_ego(initial_state=state))

replay.visualize(sleep_time=5)
executionTime = (time.time() - startTime)

print('Execution time in seconds: ' + str(executionTime))
