from tutorials.pid_tuner.utils import PidTunerParams, scenario_file, get_params, render_position_differences
import numpy as np
from carlacr.carla_interface import CarlaInterface
import scipy
from commonroad.common.file_reader import CommonRoadFileReader
from typing import List


scenario, pps = CommonRoadFileReader(scenario_file).open()

MAX_KP = 10.0
INCREASE_KP = 0.1

# Kp_lon,Ki_lon,Kd_lon -> acceleration
lon_params = [0.0, 0.0, 0.0]

# Kp_lat,Ki_lat,Kd_lat -> steering
lat_params = [1.95, 0.05, 0.2]


def get_PU(array: List[float], rtol: int = 0, atol: float = 1.0):
    """
    Calculate the ultimate period (PU) of an oscillating array.

    :param array: 1D array of oscillating values
    :param rtol: The relative tolerance parameter (see numpy.allclose)
    :param atol: The absolute tolerance parameter (see numpy.allclose)
    :return: PU value if the differences between consecutive elements in the array are close, otherwise None
    """
    diffs = np.diff(array)
    if np.allclose(diffs, diffs[0], rtol=rtol, atol=atol):
        values, counts = np.unique(diffs, return_counts=True)
        return round(values[np.argmax(counts)] * 0.05, 3)
    else:
        return None


def calculate_final_params(KU: float, PU: float):
    """
    Calculate the final parameters for a PID controller using the Ziegler-Nichols method.

    :param KU: Ultimate gain
    :param PU: Ultimate period
    :return: Dictionary containing the final values for Kp, TI and TD
    """
    return {"Kp": 0.6 * KU, "TI": PU / 2.0, "TD": PU / 8.0}


def are_oscillations_stable(
    array: List[float], percentage: int = 90, atol: float = 0.001
):
    """
    Check if the oscillations in an array are stable.

    :param array: 1D array of oscillating values
    :param percentage: Percentage of values that must be close to the mean for the oscillations to be considered stable
    :param atol: The absolute tolerance parameter (see numpy.isclose)
    :return: True if the oscillations are stable, otherwise False
    """
    close_values = np.isclose(array, np.mean(array), rtol=0, atol=atol)
    if np.mean(close_values) >= percentage / 100:
        return True
    return False


while lon_params[0] < MAX_KP:
    params = get_params(PidTunerParams(lon_params, lat_params))
    ci = CarlaInterface(params)
    ci.replay(scenario)

    obs = ci._cr_obstacles[0]

    actual_trajectory_velocity = np.array([t.velocity for t in obs.trajectory])
    actual_trajectory_velocity = actual_trajectory_velocity[1:]

    if ci.run_simulation_finished:
        velocity_differences = actual_trajectory_velocity - 20.0
        velocity_differences = velocity_differences[200:600]

        peaks, _ = scipy.signal.find_peaks(velocity_differences, prominence=0.1)
        peak_values = velocity_differences[peaks]
        print(peak_values)
        if len(peak_values) > 15 and are_oscillations_stable(peak_values):
            PU = get_PU(peaks)
            KU = lon_params[0]
            print("The calculated optimal parameters: ", calculate_final_params(KU, PU))
            break

        render_position_differences(velocity_differences, f"lon_KP:{lon_params[0]:.3f}")

    lon_params[0] = lon_params[0] + INCREASE_KP
