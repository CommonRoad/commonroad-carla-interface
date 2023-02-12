import dataclasses
import inspect
from dataclasses import dataclass, field
import pathlib
from typing import Dict, Union, List
from omegaconf import OmegaConf

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
    sleep_time: float = 5.0  # time to move your view in carla-window
    start_carla_server: bool = True
    default_carla_paths: List[str] = field(default_factory=lambda: [
        "/opt/carla-simulator/", "/~/CARLA_0.9.14_RSS/", "/~/CARLA_0.9.14/",
        "/~/CARLA_0.9.13_RSS/", "/~/CARLA_0.9.13/"])
    offscreen_mode: bool = False

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
class SimulationParams(BaseParam):
    time_step: float =  0.1
    tm_port: int = 8000  # traffic manager port
    hybrid_physics_mode: bool = False
    synchronous: bool = True
    client_init_timeout: float = 10.0
    global_percentage_speed_difference: float = 0.0
    global_distance_to_leading_vehicle: float = 1.0

@dataclass
class ControlParams(BaseParam):
    basic_control_pid_lat_kp: float = 1.95
    basic_control_pid_lat_ki: float = 0.2
    basic_control_pid_lat_kd: float = 0.07

    basic_control_pid_lon_kp: float = 1.0
    basic_control_pid_lon_ki: float = 0.0
    basic_control_pid_lon_kd: float = 0.75

    ackermann_pid_speed_kp: float = 0.15
    ackermann_pid_speed_ki: float = 0.0
    ackermann_pid_speed_kd: float = 0.25
    ackermann_pid_accel_kp: float = 0.01
    ackermann_pid_accel_ki: float = 0.0
    ackermann_pid_accel_kd: float = 0.01

@dataclass
class ObstacleParams(BaseParam):
    percentage_pedestrians_running: float =  0  # how many pedestrians will run
    percentage_pedestrians_crossing: float = 0  # how many pedestrians will walk through the road

class Mode2dParams(BaseParam):
    description: str = "CARLA No Rendering Mode Visualizer"
    res: str = '1280x720'  # window resolution (default: 1280x720)
    filter: str = 'vehicle.*'  # actor filter (default: "vehicle.*")

class MapParams(BaseParam):
    vertex_distance: float = 2.0  # in meters
    max_road_length: float = 500.0  # in meters
    wall_height: float = 1.0  # in meters
    extra_width: float = 0.6  # in meters

@dataclass
class CarlaParams(BaseParam):
    simulation: SimulationParams = field(default_factory=SimulationParams)
    control: ControlParams = field(default_factory=ControlParams)
    obstacle: ObstacleParams = field(default_factory=ObstacleParams)
    mode_2d: Mode2dParams = field(default_factory=Mode2dParams)
    map: MapParams = field(default_factory=MapParams)
