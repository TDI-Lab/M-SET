# SETTING CONSTANTS
HOVER_HEIGHT = 0.6 #m
SPEED = 0.1 #m/s
INPUT_MODE = "cdca" # CAN BE OVERWITTEN AT COMMAND LINE # "cdca" or "default"
ENABLE_LOGGING = False
PRINT_LOG_MESSAGES= False
LOG_OUTPUT_FILE = "placeholder"
IN_SIMULATION = False # Set automatically by --sim in command line
TRAVEL_TIME_MODE = 2
USE_CELL_COORDS = False # True=path uses testbed grid coords, False=path uses the actual positions from the physical testing environment
TIMESTEP_LENGTH = 0.015625#0.0078125 # Should always be a value that can be represented by 2^{x}, where x is an integer
GLOBAL_TRAVEL_TIME = 6 # s, Not required unless TRAVEL_TIME_MODE=0
SENSING_TIME=1 # s
CRAZYSWARM_SCRIPTS_FILE_PATH = "crazyswarm/ros_ws/src/crazyswarm/scripts" # Set to the full filepath (from root) of the crazyswarm/ros_ws/src/crazyswarm/scripts file in the crazyswarm installation. Needs to be set on each individual installation.