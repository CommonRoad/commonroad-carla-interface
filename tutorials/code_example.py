from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import CustomState
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile
import numpy as np


file_path = "ZAM_Test-1_1_T-1.xml"
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
dt = 0.1

# generate the states for the obstacle for time steps 1 to 40 by assuming constant velocity
state_list = []
# create the trajectory of the obstacle, starting at time step 1
dynamic_obstacle_trajectory = None
current_state = planning_problem_set.planning_problem_dict[0].initial_state
acceleration = [0] * 100000
idx = 0
while dynamic_obstacle_trajectory is None \
        or not planning_problem_set.planning_problem_dict[0].goal_reached(dynamic_obstacle_trajectory)[0]:
    a = acceleration[idx]
    # compute new position
    new_position = \
        np.array([current_state.position[0] + current_state.velocity * dt + 0.5 * scenario.dt * a * dt**2, 0])
    new_velocity = current_state.velocity + a * dt
    # create new state
    current_state = CustomState(position=new_position, velocity=new_velocity, orientation=0.00, time_step=idx + 1)
    # add new state to state_list
    state_list.append(current_state)
    dynamic_obstacle_trajectory = Trajectory(1, state_list)
    idx += 1

# create the trajectory of the obstacle, starting at time step 1
dynamic_obstacle_trajectory = Trajectory(1, state_list)

# create the prediction using the trajectory and the shape of the obstacle
dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)

# generate the dynamic obstacle according to the specification
dynamic_obstacle_id = scenario.generate_object_id()
dynamic_obstacle_type = ObstacleType.CAR
dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id,
                                   dynamic_obstacle_type,
                                   dynamic_obstacle_shape,
                                   planning_problem_set.planning_problem_dict[0].initial_state,
                                   dynamic_obstacle_prediction)

scenario.add_objects(dynamic_obstacle)

fw = CommonRoadFileWriter(scenario, planning_problem_set)

filename = "ZAM_Tutorial-1_2_T-1.xml"
fw.write_to_file(filename, OverwriteExistingFile.ALWAYS)

