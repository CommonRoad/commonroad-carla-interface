import time
import logging
import os
import sys
import typer
from carlacr.mode.carla_mode import CarlaMode

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath("__file__"))))

logger = logging.getLogger(__name__)

cli = typer.Typer(help="Generates visualization in Carla for Commonroad scenario and map")


@cli.command()
def visualize(scenario_file: str = typer.Argument(..., help="Path to scenario file"),
              map_file: str = typer.Argument(..., help="Path to map file"),
              veh_id: int = typer.Option(-1, help="id of the vehicle"),
              saving_video: str = typer.Option(None, help="path to save the video file"),
              ):
    """Generates visualization in Carla for Commonroad scenario and map."""
    start_time = time.time()
    replay = CarlaMode(cr_scenario_path=scenario_file, open_drive_map_path=map_file)
    if veh_id != -1:
        replay.set_ego_vehicle_by_id(veh_id)
    if saving_video:
        replay.visualize(sleep_time=5)
    else:
        replay.visualize(sleep_time=5)
    executiontime = (time.time() - start_time)

    logger.debug('Execution time in seconds: %e', str(executiontime))


if __name__ == "__main__":
    cli()
