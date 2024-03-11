import sys

from crcarla.helper.config import BaseParam
from crcarla.helper.utils import find_carla_distribution

base_param = BaseParam()
sys.path.append(str(find_carla_distribution(base_param.default_carla_paths) / "PythonAPI/carla"))
