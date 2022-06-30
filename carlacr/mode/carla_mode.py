import carla
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle, ObstacleRole
from commonroad.scenario.trajectory import State, Trajectory
from carlacr.interface.carla_interface import CarlaInterface


class CarlaMode:
    def __init__(self, open_drive_map_path: str,
                 cr_scenario_file_path: str = None,
                 vehicle_id: int = -1):

        self.carla_client = carla.Client("localhost", 2000)
        self.time_step_delta = None
        self.ego_vehicle = None
        if vehicle_id != -1:
            self.set_ego_vehicle_by_id(vehicle_id)

        # super().__init__(open_drive_map_path=open_drive_map_path, carla_client=self.carla_client)
        self.carla_interface = CarlaInterface(open_drive_map_path=open_drive_map_path,
                                              cr_scenario_file_path=cr_scenario_file_path,
                                              carla_client=carla.Client("localhost", 2000))

    def saving_video(self, create_video: bool = True,
                     video_path: str = None,
                     video_name: str = "test",
                     video_as_mp4: bool = False):
        """c
        :param create_video: flag for creating video
        :param video_path: path to a folder where the gif will be saved,
        additionally a folder at "gif_path"/img will be
        created in to save the images used for the gif
        :param video_name: filename for the gif
        :param video_as_mp4: flag to save as mp4 or gif
        """
        self.carla_interface.saving_video(create_video=create_video,
                                          video_path=video_path,
                                          video_name=video_name,
                                          video_as_mp4=video_as_mp4)

    def set_carla_client(self, host: str, port: int):
        """
        set up carla client view

        :param host: host address
        :param port: port number
        """
        self.carla_client = carla.Client(host, port)
        self.carla_interface.client = self.carla_client

    def set_ego_vehicle_by_id(self, veh_id: int):
        vehicle = self.obstacle_by_id(veh_id)
        if not vehicle:
            raise ValueError("There are no vehicle with the given id")
        self.set_ego_vehicle(vehicle)

    def set_ego_vehicle(self, ego_vehicle: DynamicObstacle):
        """
        set up ego_vehicle view

        :param ego_vehicle: CommonRoad vehicle
        """
        if not ego_vehicle:
            raise ValueError("ego vehicle should not be null")
        if ego_vehicle.obstacle_role != ObstacleRole.DYNAMIC:
            raise AttributeError("ego vehicle muss be dynamic")
        self.ego_vehicle = ego_vehicle

    def obstacle_by_id(self, veh_id: int):
        """

        find vehicle in scenario with commonroad ID
        :param veh_id: commonroad id of the vehicle
        :return: the vehicle if found else return None
        """

        return self.carla_interface.scenario.obstacle_by_id(veh_id)

    def create_dynamic_obstacles_ego(self, initial_state: State,
                                     trajectory: Trajectory = None) -> DynamicObstacle:
        """
        create CommonRoad moving dynamic obstacle

        :param initial_state: initial state of the dynamic obstacle
        :param trajectory: trajectory of the dynamic obstacle
        :return: static obstacle object
        """
        new_id = self.carla_interface.scenario.generate_object_id()
        shape = Rectangle(2, 4.5)
        initial_state.time_step = 1
        if not trajectory:
            trajectory = Trajectory(initial_time_step=1, state_list=[initial_state])
        prediction = TrajectoryPrediction(trajectory=trajectory, shape=shape)
        obj = DynamicObstacle(obstacle_id=new_id,
                              obstacle_type=ObstacleType.PARKED_VEHICLE,
                              obstacle_shape=Rectangle(2, 4.5),
                              initial_state=initial_state,
                              prediction=prediction)
        return obj

    def visualize(self, sleep_time: int = 10, time_step_delta_real=None):
        pass
