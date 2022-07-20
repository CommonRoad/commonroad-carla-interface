"""Parses the arguments received from commandline and runs the game loop"""
import argparse
import sys
import logging
from carlacr.configurations.set_configs import set_configs
sys.path.append(set_configs().carla_config.carla_examples_path)

logger = logging.getLogger(__name__)
try:
    from examples.no_rendering_mode import game_loop
except IndexError:
    logger.info("Can not find no_rendering_mode")
    raise


def carla_2d_mode():

    """
    Set all necessary parameters for no_rendering_mode.py
    Details please see help
    :return: None
    """
    # Load config files
    config = set_configs()

    # Define arguments that will be received and parsed
    argparser = argparse.ArgumentParser(
        description=config.config_carla_2d.description)
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default=config.carla_config.host,
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=config.carla_config.port,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default=config.config_carla_2d.res,
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default=config.config_carla_2d.filter,
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--map',
        metavar='TOWN',
        default=None,
        help='start a new episode at the given TOWN')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        help='switch off server rendering')
    argparser.add_argument(
        '--show-triggers',
        action='store_true',
        help='show trigger boxes of traffic signs')
    argparser.add_argument(
        '--show-connections',
        action='store_true',
        help='show waypoint connections')
    argparser.add_argument(
        '--show-spawn-points',
        action='store_true',
        help='show recommended spawn points')

    # Parse arguments
    args = argparser.parse_args()
    args.description = argparser.description
    args.width, args.height = [int(x) for x in args.res.split('x')]

    # Print server information
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)
    print(__doc__)

    # Run game loop
    game_loop(args)


# run start_2d_modeStart function
carla_2d_mode()
