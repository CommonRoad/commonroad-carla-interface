import os
import sys

from carlacr.helper.config import BaseParam
from carlacr.helper.utils import find_carla_distribution


base_param = BaseParam()
sys.path.append(os.path.join(find_carla_distribution(base_param.default_carla_paths), "PythonAPI/carla"))

