import dataclasses
import inspect
from dataclasses import dataclass, field
import pathlib
from typing import Dict, Union, List
from omegaconf import OmegaConf
from enum import Enum

class PedestrianControlType(Enum):
    AI = 1
    WALKER = 2
    TRANSFORM = 3

class VehicleControlType(Enum):
    KEYBOARD = 1
    STEERING_WHEEL = 2
    TRANSFORM = 3
    PID = 4
    ACKERMANN = 5
    PATH_TM = 6
    PATH_AGENT = 7
    PLANNER = 8

class CustomVis(Enum):
    BIRD = 0
    EGO = 1
    NONE = 2


class ApproximationType(Enum):
    """Approximation type with fix length, width and area."""
    LENGTH = 0
    WIDTH = 1
    AREA = 2


def _dict_to_params(dict_params: Dict, cls):
    """

    """
    fields = dataclasses.fields(cls)
    cls_map = {f.name: f.type for f in fields}
    kwargs = {}
    for k, v in cls_map.items():
        if k not in dict_params:
            continue
        if inspect.isclass(v) and issubclass(v, BaseParam):
            kwargs[k] = _dict_to_params(dict_params[k], cls_map[k])
        else:
            kwargs[k] = dict_params[k]
    return cls(**kwargs)


@dataclass
class BaseParam:
    host: str = "localhost"  # carla host setting
    port: int = 2000  # carla default port setting
    sleep_time: float = 10.0  # time to move your view in carla-window
    start_carla_server: bool = True
    default_carla_paths: List[str] = field(default_factory=lambda: [
        "/opt/carla-simulator/", "/~/CARLA_0.9.14_RSS/", "/~/CARLA_0.9.14/",
        "/~/CARLA_0.9.13_RSS/", "/~/CARLA_0.9.13/"])
    offscreen_mode: bool = True
    map: str = "Town01"
    client_init_timeout: float = 30.0
    sync: bool = True
    autopilot: bool = False
    vis_type: CustomVis = CustomVis.BIRD
    __initialized: bool = field(init=False, default=False, repr=False)

    def __post_init__(self):
        self.__initialized = True
        # Make sure that the base parameters are propagated to all sub-parameters
        # This cannot be done in the init method, because the sub-parameters are not yet initialized.
        # This is not a noop, as it calls the __setattr__ method.
        # Do not remove!
        self.host = self.host
        self.port = self.port
        self.sleep_time = self.sleep_time
        self.start_carla_server = self.start_carla_server
        self.default_carla_paths = self.default_carla_paths
        self.offscreen_mode = self.offscreen_mode
        self.client_init_timeout = self.client_init_timeout
        self.map = self.map
        self.sync = self.sync
        self.autopilot = self.autopilot
        self.vis_type = self.vis_type

    def __getitem__(self, item):
        try:
            value = self.__getattribute__(item)
        except AttributeError:
            raise KeyError(f"{item} is not a parameter of {self.__class__.__name__}")
        return value

    def __setitem__(self, key, value):
        try:
            self.__setattr__(key, value)
        except AttributeError:
            raise KeyError(f"{key} is not a parameter of {self.__class__.__name__}")

    @classmethod
    def load(cls, file_path: Union[pathlib.Path, str], validate_types: bool = True):
        file_path = pathlib.Path(file_path)
        assert file_path.suffix == ".yaml", f"File type {file_path.suffix} is unsupported! Please use .yaml!"
        loaded_yaml = OmegaConf.load(file_path)
        if validate_types:
            OmegaConf.merge(OmegaConf.structured(CarlaParams), loaded_yaml)
        params = _dict_to_params(OmegaConf.to_object(loaded_yaml), cls)
        return params

    def save(self, file_path: Union[pathlib.Path, str]):
        # Avoid saving private attributes
        dict_cfg = dataclasses.asdict(self, dict_factory=lambda items: {key: val for key, val in items if
                                                                        not key.startswith("_")})
        OmegaConf.save(OmegaConf.create(dict_cfg), file_path, resolve=True)


@dataclass
class TrafficManagerParams(BaseParam):
    # port where the traffic manager is connected
    tm_port: int = 8000
    # vehicle's farther than a certain radius from the ego vehicle will have their physics disabled
    hybrid_physics_mode: bool = False
    # radius of the area where physics are enabled
    hybrid_physics_radius: float = 70.0
    # difference of the vehicle's intended speed and its current speed limit [%]
    # exceeding a speed limit can be done using negative percentage
    global_percentage_speed_difference: float = 0.0
    # minimum distance that vehicles have to keep with the other vehicles [m]
    # computed from center to center
    global_distance_to_leading_vehicle: float = 1.0
    # random seed for the traffic manager
    seed: int = 0
    # allows having dead-end streets; Normally, if vehicles cannot find the next waypoint, TM crashes.
    # If OSM mode is enabled, it will show a warning, and destroy vehicles when necessary.
    osm_mode: bool = False
    # how many pedestrians will run [%]
    global_percentage_pedestrians_running: float = 0
    # how many pedestrians will walk through the road [%]
    global_percentage_pedestrians_crossing: float = 0
    # global lane offset displacement from the center line [%]
    # Positive values imply a right offset while negative ones mean a left one.
    global_lane_offset: float = 0.0
    # collisions with walkers will be ignored for a vehicle [%]
    ignore_walkers_percentage: float = 0.0
    # chance that vehicle will follow the keep right rule, and stay in the right lane [%]
    keep_right_rule_percentage: float = 0.0
    # chance that collisions with another vehicle will be ignored for a vehicle [%]
    ignore_vehicles_percentage: float = 0.0
    # chance that stop signs will be ignored for a vehicle [%]
    ignore_signs_percentage: float = 0.0
    # chance that traffic lights will be ignored for a vehicle [%]
    ignore_lights_percentage: float = 0.0
    # probability that actor will perform a left lane change, dependent on lane change availability [%]
    random_left_lane_change_percentage: float = 0.0
    # probability that actor will perform a right lane change, dependent on lane change availability [%]
    random_right_lane_change_percentage: float = 0.0


@dataclass
class SimulationParams(BaseParam):
    tm: TrafficManagerParams = field(default_factory=TrafficManagerParams)
    time_step: float =  0.1
    max_substep_delta_time: float = 0.01
    max_substeps: int = 10
    number_walkers: int = 10
    number_vehicles: int = 30
    safe_vehicles: bool = True
    filter_vehicle: str = "vehicle.*"
    filter_pedestrian: str = 'walker.pedestrian.*'
    seed_walker: int = 0
    pedestrian_default_shape: bool = False
    max_time_step: int = 60
    width: float = 1280
    height: float = 720
    rolename: str = "hero"
    generation: str = "2"
    gamma: float = 2.2
    description: str = "Keyboard Control"
    show_triggers: bool = True
    show_connections: bool = True
    show_spawn_points: bool = True
    record_video: bool = False
    video_path: str = "./"
    video_name: str = "CommonRoad"
    vis_hud: bool = True


@dataclass
class ControlParams(BaseParam):
    basic_control_pid_lat_kp: float = 1.95
    basic_control_pid_lat_ki: float = 0.05
    basic_control_pid_lat_kd: float = 0.2

    basic_control_pid_lon_kp: float = 1.0
    basic_control_pid_lon_ki: float = 0.05
    basic_control_pid_lon_kd: float = 0.0

    ackermann_pid_speed_kp: float = 0.15
    ackermann_pid_speed_ki: float = 0.0
    ackermann_pid_speed_kd: float = 0.25
    ackermann_pid_accel_kp: float = 0.01
    ackermann_pid_accel_ki: float = 0.0
    ackermann_pid_accel_kd: float = 0.01

    def pid_lat_dict(self, dt: float) -> Dict[str, float]:
        return {"K_P": self.basic_control_pid_lat_kp,
                "K_I": self.basic_control_pid_lat_ki,
                "K_D": self.basic_control_pid_lat_kd,
                "dt": dt}

    def pid_lon_dict(self, dt: float) -> Dict[str, float]:
        return {"K_P": self.basic_control_pid_lon_kp,
                "K_I": self.basic_control_pid_lon_ki,
                "K_D": self.basic_control_pid_lon_kd,
                "dt": dt}

@dataclass
class VehicleParams(BaseParam):
    approximation_type: ApproximationType = ApproximationType.LENGTH  # based on what approximation of the vehicle
    # size the blueprint should be selected
    physics: bool = True  # if physics should be enabled for the vehicle
    control: ControlParams = field(default_factory=ControlParams)
    simulation: SimulationParams = field(default_factory=SimulationParams)
    vehicle_ks_state: bool = True
    path_sampling: int = 10 # use every path_sampling time step for path to follow CR trajectory
    controller_type: VehicleControlType = VehicleControlType.TRANSFORM

class PedestrianParams(BaseParam):
    # size the blueprint should be selected
    physics: bool = True  # if physics should be enabled for the vehicle
    simulation: SimulationParams = field(default_factory=SimulationParams)
    controller_type: PedestrianControlType = PedestrianControlType.TRANSFORM


@dataclass
class MapParams(BaseParam):
    vertex_distance: float = 2.0  # in meters
    max_road_length: float = 500.0  # in meters
    wall_height: float = 1.0  # in meters
    extra_width: float = 0.6  # in meters


@dataclass
class CarlaParams(BaseParam):
    simulation: SimulationParams = field(default_factory=SimulationParams)
    pedestrian: PedestrianParams = field(default_factory=PedestrianParams)
    vehicle: VehicleParams = field(default_factory=VehicleParams)
    ego: VehicleParams = field(default_factory=VehicleParams)
    map_params: MapParams = field(default_factory=MapParams)
