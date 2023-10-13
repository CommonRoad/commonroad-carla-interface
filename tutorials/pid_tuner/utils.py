# longitudinal pid controller -> responsible for the speed
from dataclasses import dataclass
import os
from tests.utils import make_output_dir
from carlacr.helper.config import (
    CarlaParams,
    ControlParams,
    VehicleParams,
    VehicleControlType,
)
import matplotlib.pyplot as plt

file_dir = os.path.dirname(os.path.realpath(__file__))


scenario_file = os.path.join(file_dir,  "/../../scenarios/ZAM_Tutorial-1_2_T-1.xml")
map_file =  os.path.join(file_dir, "/../../maps/ZAM_Test-1_2_T-1.xodr")

output_dir = make_output_dir()


@dataclass
class PidTunerParams:
    lon_kp: float = 0
    lon_ki: float = 0
    lon_kd: float = 0

    lat_kp: float = 0
    lat_ki: float = 0
    lat_kd: float = 0

    def __init__(self, lon_array, lat_array):
        self.lon_kp = lon_array[0]
        self.lon_ki = lon_array[1]
        self.lon_kd = lon_array[2]

        self.lat_kp = lat_array[0]
        self.lat_ki = lat_array[1]
        self.lat_kd = lat_array[2]


def get_params(pid_params: PidTunerParams):
    params = CarlaParams()
    params.start_carla_server = False
    params.offscreen_mode = True
    params.sync = True
    params.map = map_file
    params.start_carla_server = False
    params.ego = 47

    controll_params = ControlParams()
    controll_params.basic_control_pid_lat_kd = pid_params.lat_kd
    controll_params.basic_control_pid_lat_ki = pid_params.lat_ki
    controll_params.basic_control_pid_lat_kp = pid_params.lat_kp

    controll_params.basic_control_pid_lon_kd = pid_params.lon_kd
    controll_params.basic_control_pid_lon_ki = pid_params.lon_ki
    controll_params.basic_control_pid_lon_kp = pid_params.lon_kp

    vehicle_params = VehicleParams()
    vehicle_params.control = controll_params
    params.vehicle = vehicle_params
    params.vehicle.carla_controller_type = VehicleControlType.PID

    return params


def render_position_differences(distances, title):
    plt.clf()
    plt.cla()

    plt.plot(distances)
    plt.xlabel("Speed Difference")
    plt.ylabel("Time")

    plt.ylim(-20, 20)

    filename = f"{title}.png"
    filepath = os.path.join(output_dir, filename)
    plt.title(title)
    plt.savefig(filepath)
