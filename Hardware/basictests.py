import unittest
from cdca_epos_executor import *

# SETTING CONSTANTS
HOVER_HEIGHT = 0.5 #m
SPEED = 0.1 #m/s
INPUT_MODE = "cdca"
IN_SIMULATION = True # Should be set automatically by --sim in command line
TRAVEL_TIME_MODE = 2
USE_CELL_COORDS = True
TIMESTEP_LENGTH = 1
GLOBAL_TRAVEL_TIME = 6 # Not required unless TRAVEL_TIME_MODE=0
CRAZYSWARM_SCRIPTS_FILE_PATH = "/home/adam/Documents/Packages/crazyswarm/ros_ws/src/crazyswarm/scripts" # Needs to be set on individual system

# GLOBAL VARIABLES
# input_file_path = ... # command line?
# #os.chdir(CRAZYSWARM_SCRIPTS_FILE_PATH) # Needs to be here in the order
# swarm = ...
# timeHelper = ...
# allcfs = ...
# input_path = ...
# all_drones = ...
# next_moves = ...

# NOT SURE IF NEEDED
sys.path.append(CRAZYSWARM_SCRIPTS_FILE_PATH) # append a new directory to sys.path
from pycrazyswarm import Crazyswarm # KEEP

class TestCdcaEposExecutor (unittest.TestCase):

    def setUp(self):
        self.input_file_path = "epospaths/Testpaths/onenice.txt"
        if INPUT_MODE == "cdca":
            self.input_path = read_cdca_output(self.input_file_path)
        elif INPUT_MODE == "default":
            self.input_path = read_default_output(self.input_file_path)
        os.chdir(CRAZYSWARM_SCRIPTS_FILE_PATH)
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs
        self.all_drones, self.next_moves = parse_input(self.input_path, self.allcfs, INPUT_MODE, SPEED, next_moves=np.array([]))

        take_off_all(HOVER_HEIGHT, 2.5, self.timeHelper, self.all_drones)
        set_initial_positions(self.timeHelper, self.all_drones, USE_CELL_COORDS, INPUT_MODE)

    def tearDown(self):
        land_all(HOVER_HEIGHT, 0.05, self.timeHelper, self.all_drones)

    def test_example(self):
        print(self.input_file_path, self.input_path, self.timeHelper, self.allcfs, self.input_path, self.all_drones, self.next_moves)

    def test_follow_plan(self):
        follow_plans(self.timeHelper, self.all_drones, self.next_moves, TRAVEL_TIME_MODE, USE_CELL_COORDS, 0, GLOBAL_TRAVEL_TIME, INPUT_MODE, TIMESTEP_LENGTH, IN_SIMULATION)

test = TestCdcaEposExecutor()
test.setUp()
test.test_example()
test.test_follow_plan()

#main(IN_SIMULATION, INPUT_MODE, input_file_path, TRAVEL_TIME_MODE, USE_CELL_COORDS, 0, HOVER_HEIGHT, SPEED, TIMESTEP_LENGTH=1, GLOBAL_TRAVEL_TIME=6)
#main(True, "cdca", "epospaths/Testpaths/onenice.csv", 2, True, 0, 0.5, 0.1, 1)

# Reading input
# Setting up Crazyswarm variables
# Setting up simulation variables - drones, next_moves, uris
# Pre-simulation logging
# Initialise simulation - move to initial positions, take off
# Follow paths
# Close simulation - landing
# Post-simulation logging