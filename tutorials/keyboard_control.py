from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Tag

from carlacr.carla_interface import CarlaInterface
from carlacr.helper.config import CarlaParams, CustomVis, EgoPlanner

# Keyboard control with storing of driven scenario

# Configure simulation and scenario settings
param = CarlaParams()
param.vehicle.vehicle_ks_state = True
param.offscreen_mode = True  # set to false if your system is powerful enough
param.vis_type = CustomVis.BIRD  # set to EGO if your system is powerful enough
param.simulation.number_vehicles = 2
param.simulation.number_walkers = 2
param.simulation.max_time_step = 120

# Initialize CARLA-Interface and start keyboard control
ci = CarlaInterface(param)
ci.external_control(EgoPlanner.KEYBOARD)

# Store generated scenario
sc = ci.create_cr_scenario()
CommonRoadFileWriter(
    sc,
    PlanningProblemSet(),
    author="TUM-CPS",
    affiliation="Technical University of Munich",
    source="CARLA",
    tags={Tag.URBAN},
).write_to_file(None, OverwriteExistingFile.ALWAYS)
