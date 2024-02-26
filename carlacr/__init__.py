import sys

from carlacr.helper.config import BaseParam
from carlacr.helper.utils import find_carla_distribution

base_param = BaseParam()
sys.path.append(str(find_carla_distribution(base_param.default_carla_paths) / "PythonAPI/carla"))
