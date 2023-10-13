from dataclasses import dataclass
from typing import Any, Callable, Optional
import datetime
import json
from carlacr.helper.config import BaseParam
from carlacr.helper.utils import find_carla_distribution, kill_existing_servers
import os
import subprocess
import time

TEST_CASE_PORT = 2001


@dataclass
class TCase:
    """Class for test cases in traffic light cycle creation tests."""

    expected: Any
    input: Optional[Any] = None
    function: Optional[Callable] = None
    output: Optional[Any] = None

    def __post_init__(self):
        if not hasattr(self.expected, "__eq__"):
            raise TypeError(f"{type(self.expected).__name__} does not implement the __eq__ method")

        if self.input is not None and self.function is not None:
            self.output = self.function(*self.input)


def make_log_dir():
    now = datetime.datetime.now()
    dir_name = now.strftime("%Y-%m-%d_%H-%M-%S")
    logs_dir = os.path.join(os.getcwd(), "logs")
    os.makedirs(logs_dir, exist_ok=True)
    dir_path = os.path.join(logs_dir, dir_name)
    os.mkdir(dir_path)
    return dir_path


def make_output_dir():
    now = datetime.datetime.now()
    dir_name = now.strftime("%Y-%m-%d_%H-%M-%S")
    logs_dir = os.path.join(os.getcwd(), "output")
    os.makedirs(logs_dir, exist_ok=True)
    dir_path = os.path.join(logs_dir, dir_name)
    os.mkdir(dir_path)
    return dir_path


def compare_json(file1, file2):
    with open(file1, "r") as f:
        data1 = json.load(f)
    with open(file2, "r") as f:
        data2 = json.load(f)
    diff = {}
    for key in data1.keys():
        if key not in data2:
            diff[key] = (data1[key], "<Key not found>")
        elif data1[key] != data2[key]:
            diff[key] = (data1[key], data2[key])
    for key in data2.keys():
        if key not in data1:
            diff[key] = ("<Key not found>", data2[key])
    return diff


def start_carla_test_case_server():
    _p = BaseParam()

    kill_existing_servers(_p.sleep_time)

    path_to_carla = os.path.join(
        find_carla_distribution(_p.default_carla_paths), "CarlaUE4.sh"
    )

    popen_base_params = {
        "stdout": subprocess.PIPE,
        "preexec_fn": os.setsid,
        "shell": False,
    }

    cmd = [path_to_carla, "-RenderOffScreen"]
    subprocess.Popen(cmd, **popen_base_params)

    time.sleep(_p.sleep_time)
