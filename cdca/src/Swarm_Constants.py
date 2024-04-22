# Swarm constants definition.
import math

SPEED = 0.1
TIME_STEP = 1

DISTANCE_STEP = SPEED * TIME_STEP 

MINIMUM_DISTANCE = 0.2 # Must be >= to DISTANCE_STEP to ensure accuracy

TIME_DELAY = (MINIMUM_DISTANCE / SPEED) + TIME_STEP

COLLISION_AVOIDANCE_LIMIT = 20

COMPARED_FLIGHT = 2
SUBJECT_FLIGHT = 1

COMPARED_DRONE = 2
SUBJECT_DRONE = 1

EPOS_TIMESTEP = 1

CROSS_COLLISION = 1
PARALLEL_COLLISION = 2
DEST_OCCUPIED_COLLISION = 3
SAME_FINAL_POSITIONS_COLLISION = 4

MIN_POINTS_PARALLEL_COLLISION = math.ceil(7 / DISTANCE_STEP)

PARALLEL_CA_ORTH_DIST = MINIMUM_DISTANCE * 1.5

INTERPOLATION_FACTOR = 10 #FPS
FRAMES_PER_TIMESTEP = INTERPOLATION_FACTOR * TIME_STEP

GRID_SIZE = 10
MIN_GRID_OFFSET = -5
MAX_GRID_OFFSET = 5

