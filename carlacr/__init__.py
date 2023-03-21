import os
import sys
from typing import List
from carlacr.helper.config import BaseParam

def find_path(default_carla_paths: List[str]):
    if default_carla_paths is None:
        default_carla_paths = BaseParam().default_carla_paths
    for default_path in default_carla_paths:
        path = default_path.replace("/~", os.path.expanduser("~"))
        if os.path.exists(path):
            return path
    raise FileNotFoundError("CARLA executable not found.")

base_param = BaseParam()
sys.path.append(os.path.join(find_path(base_param.default_carla_paths), "PythonAPI/carla"))

