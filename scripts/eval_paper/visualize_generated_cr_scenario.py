from pathlib import Path

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

file_path = "/home/sebastian/Downloads/ZAM_CARLATown10-1.xml"
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

for timeStep in [0, 55]:
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    rnd.draw_params.axis_visible = False
    rnd.draw_params.dynamic_obstacle.trajectory.draw_trajectory = False
    rnd.draw_params.lanelet_network.lanelet.draw_start_and_direction = False
    rnd.draw_params.lanelet_network.lanelet.draw_line_markings = False
    rnd.draw_params.dynamic_obstacle.draw_icon = True
    rnd.draw_params.time_begin = timeStep
    scenario.draw(rnd)
    rnd.render()
    plt.ylim((-49, 12))
    plt.xlim((-84, -4.5))
    plt.savefig(str(Path(__file__).parent / f"ZAM_CARLATown10-1-TimeStep{timeStep}.svg"))
