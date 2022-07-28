import sys
from carlacr.configurations.set_configs import set_configs
sys.path.append(set_configs().carla_config.carla_agent_path)
