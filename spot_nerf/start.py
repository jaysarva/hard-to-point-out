from arm_simple import hello_arm
from colmap2nerf import start_colmap
from run import runnerf
from coordinate_transform import transform
import bosdyn.client
import argparse
import sys

def main(argv):
    """Command line interface."""
    #parser = argparse.ArgumentParser()
    #spot start moving and taking images
    #start_spot(parser, argv)
    #once the spot has collected some images, we run colmap
    start_colmap()
    #after colmap gets us the coordinate, we need to transform to the coordinate system we can understand in the real world
    transform()
    #once we get the coord and images, we start running nerf
    runnerf(gui=True)
    exit(0)





def start_spot(parser, argv):
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)
    try:
        hello_arm(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception("Threw an exception")
        return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
