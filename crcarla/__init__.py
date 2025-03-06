import sys
import argparse
from crcarla.helper.config import BaseParam, SupportedCARLAVersion
from crcarla.helper.utils import find_carla_distribution

parser = argparse.ArgumentParser(description='CARLA-Interface')
parser.add_argument('--cvers', action="store", dest='cvers', default="0.9.15", type=str, help='CARLA version to use.')
args = parser.parse_args()

base_param = BaseParam()
sys.path.append(str(find_carla_distribution(base_param.default_carla_paths, SupportedCARLAVersion(args.cvers)) / "PythonAPI/carla"))
