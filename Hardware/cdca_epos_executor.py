import numpy as np
import os
import sys
import math
from decimal import Decimal
import rospy
from std_msgs.msg import String

try:
    from Hardware_constants import *
    from ROSListener import *
except:
    from Hardware.Hardware_constants import *
    from Hardware.ROSListener import * 

# Append a new directory to sys.path
sys.path.append(CRAZYSWARM_SCRIPTS_FILE_PATH)
from pycrazyswarm import Crazyswarm
Z=HOVER_HEIGHT

pub = rospy.Publisher('status_logger', String, queue_size=10)

# Translate the positions given in the path into 3D coordinates in the relevant coordinate space
def get_coords(position, use_cell_coords):
    if use_cell_coords == True:
        x = convert_coords(position[0],"x")
        y = convert_coords(position[1],"y")
        z = HOVER_HEIGHT
        return([x,y,z])
    else:
        return [position[0],position[1],HOVER_HEIGHT]

# Convert coordinates from the physical coordinate space into the coordinates given relative to the positions on the testbed grid
def convert_coords(val, axis):
    if axis == "x":
        return (val-2)*0.553
    elif axis == "y":
        return ((2*val)-3)*0.235
    else:
        return NameError

# Read drone path (to which collision avoidance has been applied) from the given file
def read_cdca_output(filename):
    try:
        f = open(filename,"r")
    except FileNotFoundError:
        print("Error: FileNotFoundError")
        return []
    except:
        print("Error: Something unknown went wrong when attempting to access the file (not FileNotFoundError)")
        return []
    
    input_path = eval(f.readline())

    return input_path

# Read drone path (to which no collision avoidance has been applied) from the given file
def read_default_output(filename):
    try:
        f = open(filename,"r")
    except FileNotFoundError:
        print("Error: FileNotFoundError")
        return []
    except:
        print("Error: Something unknown went wrong when attempting to access the file (not FileNotFoundError)")
        return []
    
    # set up path array
    input_path = []

    i=0
    for line in f.readlines():
        line = line[2:] # Remove the drone number
        eval_line = eval(line)
        input_path.append([])
        for j in range(0,len(eval_line)):
            input_path[i].append(eval_line[j])
        i+=1

    return input_path

# Round to the nearest 'base' e.g. nearest 5
def round_nearest(number, base):
    return base * round(number/base)

# Round UP to the nearest 'base' e.g. nearest 5
def roundup_nearest(number, base):
    # if remainder of number/base is 0 (base divides number exactly), then don't round up further
    #   e.g. roundup_nearest(25,5) = 5, NOT 6

    diff = (number/base) - round(number/base) # diff>0 if rounded down, diff<0 if rounded up, diff=0 if exact division

    if diff > 0:
        return max(base * round(number/base), base)
    else:
        return max(base * (round(number/base) + 1), base)

# Class containing the properties of each drone in use
class Drone():
    def __init__(self, cf, speed):
        self.positions = [] # set of positions visited by the drone as part of the given path
        self.times = [] # the times that the drone spends completing each action (moving, hovering/waiting)
        self.status = "idle" # idle -> hovering -> ([sensing or waiting] moving)
        self.speed = speed # horizontal speed of the drone
        self.cf = cf # Crazyswarm crazyflie object which this drone represents
        self.id = self.cf.id # id of the drone
        self.move_count = 0 # Count of moves completed by the drone

    # Move the drone to the next position (/cell) in its path
    def move_next_cell(self, use_cell_coords, i):
        self.log_status(msg="%s moving to %s" % (i, self.positions[self.move_count]))
        pos = get_coords(self.positions[self.move_count],use_cell_coords)
        if TRAVEL_TIME_MODE == 0:
            travel_time = GLOBAL_TRAVEL_TIME # use the constant travel duration mode
        elif TRAVEL_TIME_MODE > 0:
            travel_time = self.calc_travel_time()
        else:
            print("ERROR: Invalid value for TRAVEL_TIME_MODE.")
            return -1

        self.cf.goTo(pos, 0, travel_time)
        self.move_count += 1

        # Alter the return value if needed
        if TRAVEL_TIME_MODE == 1:
            travel_time = roundup_nearest(travel_time, TIMESTEP_LENGTH)
        elif TRAVEL_TIME_MODE == 2:
            pass
        elif TRAVEL_TIME_MODE == 3:
            travel_time = roundup_nearest(travel_time, TIMESTEP_LENGTH)

        return (travel_time + TIMESTEP_LENGTH)
    
    # Calculate the duration of the movement being made by the drone
    def calc_travel_time(self):
        x_dist = (get_coords(self.positions[self.move_count], USE_CELL_COORDS)[0]) - (get_coords(self.positions[self.move_count-1], USE_CELL_COORDS)[0])
        y_dist = (get_coords(self.positions[self.move_count], USE_CELL_COORDS)[1]) - (get_coords(self.positions[self.move_count-1], USE_CELL_COORDS)[1])

        dist = math.sqrt((x_dist**2 + y_dist**2))

        time = dist / self.speed

        return time
    
    # Land the drones
    def land_drone(self, timeHelper): 
        # NB: This action is performed differently in simulation vs. on real hardware
        if IN_SIMULATION == False:
            # Calling the land command, even on just one drone, makes all of them disappear from the simulation view
            #   Therefore, this can only be done when not in simulation
            self.cf.land(0.09, 2.5)
        else: 
            # This is a workaround to avoid calling land command in simulation. The code below performs same functionality as the land command in this instance but does not turn off the motors.
            land_pos = get_coords(self.positions[self.move_count-1],USE_CELL_COORDS)
            self.cf.goTo((land_pos[0],land_pos[1],0.05),0,2.5)
        timeHelper.sleep(2.5)

    # Log the status of the drone (whether it is idle, hovering, moving, etc. See the status attribute of the Drone class)
    def log_status(self, msg=""):
        if msg != "" and PRINT_LOG_MESSAGES==True:
            print(msg)
        
        if ENABLE_LOGGING == True:
            try:
                pub.publish("Drone %s: %s. %s" % (self.id, self.status, msg))
            except rospy.ROSException:
                print("WARNING: Log failed with ROSException error. Check that ROS is running.")

# Take the positions and timings given in the drone path, and store these in the relevant Drone objects
def parse_input(input_path, allcfs, speed, next_moves):
    all_drones = []

    #parse the input
    c = 0
    for drone in input_path:
        if c < len(allcfs.crazyflies):
            d = Drone(allcfs.crazyflies[c],speed)
            all_drones.append(d)
            for position in drone:
                if INPUT_MODE == "cdca":
                    d.positions.append(position[0])
                    d.times.append(float(position[1]))
                elif INPUT_MODE == "default":
                    d.positions.append(position)
                    d.times.append(0)
            
            next_moves = np.append(next_moves, 0) # Queue 0 so that the drone immediately seeks its next action

        c+=1

    return all_drones, next_moves

# Take off all drones
def take_off_all(dur, timeHelper, all_drones, all_cfs=None, sequential=False):
    if sequential == True:
        if all_cfs == None:
            all_cfs = [drone.cf for drone in all_drones]
        all_cfs.takeoff(targetHeight=HOVER_HEIGHT, duration=dur)
        timeHelper.sleep(2.5)

        for drone in all_drones:
            drone.status = "hovering"
        log_all_drones(all_drones)
    
    else:
        # Tell the drones to take off one at a time
        for drone in all_drones:
            drone.cf.takeoff(targetHeight=HOVER_HEIGHT, duration=dur)
            timeHelper.sleep(2.5)
            drone.status = "hovering"
            drone.log_status(msg="Drone %s taken off" % drone.id)

# Land all drones
def land_all(d, timeHelper,all_drones):
# Tell the drones to take off
    for drone in all_drones:
        drone.cf.land(0.09, 2.5)
        timeHelper.sleep(2.5)

# Set the initial positions of the drones in the simulation
def set_initial_positions(timeHelper, all_drones, duration):
    for drone in all_drones:
        pos = get_coords(drone.positions[drone.move_count], USE_CELL_COORDS)
        drone.status="moving"
        drone.log_status("Drone %s moving to %s" % (drone.id, pos))
        drone.cf.goTo(pos,0,duration)
        timeHelper.sleep(duration)
        drone.status="hovering"
        drone.log_status("Drone %s reached initial position %s" % (drone.id, pos))

# Return the uris of the drones, given their ids and the radio channels that they are running on
def return_uris(channels,numbers):
    uris = []
    for i in range(0,len(channels)):
        if numbers[i] < 10:
            uris.append("radio://0/"+str(channels[i])+"/2M/E7E7E7E7"+"0"+str(numbers[i]))
        else:
            uris.append("radio://0/"+str(channels[i])+"/2M/E7E7E7E7"+str(numbers[i]))
    return uris

# Initialise logging
def init_logging():
    if ENABLE_LOGGING == True:
        print("INITIALISING LOGGING")
        global pub

        try:
            rospy.init_node('chatter', anonymous=True)
        except:
            pass

        pub = rospy.Publisher('status_logger', String, queue_size=10)

        input("Logging setup complete\nPress any key to continue")

# Log the status of all drones
def log_all_status(all_drones,msg=""):
    if ENABLE_LOGGING == True:
        for drone in all_drones:
            drone.log_status(msg=msg)

def log_all_drones(ids, vars):
    if IN_SIMULATION == False:
        if ENABLE_LOGGING == True:
            call_once(ids)
    pass
        
def adjust_moves(next_moves):
    if TRAVEL_TIME_MODE == 2:
        if np.any((next_moves < TIMESTEP_LENGTH) & (next_moves > 0)):
            for j in range(0,len(next_moves)):
                if (next_moves[j] < TIMESTEP_LENGTH and next_moves[j] > 0):
                    # round up the travel_time of the subject drone
                    next_moves[j] = roundup_nearest(next_moves[j],TIMESTEP_LENGTH)
                else:
                    # add one to the action time of the other drones
                    next_moves[j] = next_moves[j] + TIMESTEP_LENGTH

    return next_moves

# Execute the path on the drones
# Increment through the time slots of length TIMESLOT_LENGTH
# If a drone is scheduled to perform a new action (e.g. start moving, start hovering, etc.) at that time slot according to the path, execute this action
def follow_plans(timeHelper, all_drones, next_moves):
    log_all_status(all_drones, msg="Starting to follow plans")

    t=0 # timeslot counter
    while np.any(next_moves > -1):

        in_position = all(drone.status == "idle" for drone in all_drones)

        # Cycle through each drone in use
        for i in range(0,len(all_drones)):

            cf = all_drones[i]

            # if it's time for the drone to change status (i.e. it has finished its current task)
            if round_nearest(next_moves[i], TIMESTEP_LENGTH) == 0:  

                # LOG FINISHED ACTION
                cf.log_status()

                # CHECK IF DRONE REACHED END OF PATH
                if cf.move_count >= len(cf.times): 
                    # if that was the last position, mark the drone as finished
                    next_moves[i] = -1
                    
                    cf.status = "hovering"
                    cf.log_status(msg="Drone %s reached end of path" % i)
                    
                    cf.land_drone(timeHelper)
                    cf.status = "idle"
                    cf.log_status(msg="Drone %s landed" % i)

                    # LOG FINISHED ACTION
                    cf.log_status()

                # OTHERWISE, CONTINUE
                else:

                    # ASSIGN STATUS CHANGES
                    if cf.status == "moving" or cf.status == "idle" or cf.status=="hovering":
                        if (in_position == True) or (TRAVEL_TIME_MODE != 3):
                            if INPUT_MODE == "cdca":
                                # if input_move is cdca, then waiting phase follows movement phase
                                cf.status = "waiting"

                                # If waiting time is 0 then move straight to moving phase (again)
                                if (cf.times[max(0,cf.move_count - 1)]) == 0:
                                    cf.status = "moving"

                            else:
                                # Only cdca has waiting phase, so move to sensing phase in all other cases
                                cf.status = "sensing"
                                
                                # If sensing_time is 0, then move straight to moving phase (again)
                                if SENSING_TIME == 0:
                                    cf.status == "moving"

                        else:
                            # If not ready to move from idle phase, then remain idle
                            cf.status = "idle"
                            next_moves[i] = TIMESTEP_LENGTH
                    
                    elif cf.status == "waiting" or cf.status == "sensing":
                        cf.status = "moving"

                    # PERFORM ACTIONS OF (new) CURRENT PHASE                
                    if cf.status == "sensing":
                        cf.log_status(msg="Drone %s sensing" % i)
                        next_moves[i] = SENSING_TIME + TIMESTEP_LENGTH

                    elif cf.status == "waiting":
                        cf.log_status(msg="%s waiting for %s" % (i, cf.times[cf.move_count - 1]))
                        next_moves[i] = cf.times[cf.move_count - 1] + TIMESTEP_LENGTH

                    elif cf.status == "moving":
                        next_moves[i] = cf.move_next_cell(USE_CELL_COORDS, i)

                    elif cf.status == "idle":
                        pass

        # minus 1 TIMESTEP_LENGTH from all next_moves (represents 1 TIMESTEP_LENGTH passing)
        next_moves = next_moves - np.full((1,len(all_drones)),TIMESTEP_LENGTH)[0]

        # increment timeslot
        timeHelper.sleep(TIMESTEP_LENGTH)
        t = round_nearest(t + TIMESTEP_LENGTH, TIMESTEP_LENGTH)

    # Give some extra time so that the simulation doesn't shut down abruptly as soon as the drones stop moving
    log_all_status(all_drones, msg="End of simulation")
    timeHelper.sleep(3)


def main(plan, raw=False, travel_time_mode=2, use_cell_coords=True, sensing_time=0, timestep_length=1, global_travel_time=6, run=True, in_simulation=IN_SIMULATION, input_mode=INPUT_MODE):
    # Allows INPUT_MODE to be overwritten by supplying a new value (as opposed to using the one from the config file)
    global INPUT_MODE
    global IN_SIMULATION
    INPUT_MODE = input_mode
    IN_SIMULATION = in_simulation

    print("PARSING INPUT")
    if raw == True:
        input_path = plan
    else:
        if INPUT_MODE == "cdca":
            input_path = read_cdca_output(plan)
        elif INPUT_MODE == "default":
            input_path = read_default_output(plan)

    # Change directory to the crazyswarm/scripts folder
    # Required to access crazyswarm source files, since Crazyswarm assumes it is being run from a file in the crazyswarm/ros_ws/src/crazyswarm/scripts folder
    try:
        os.chdir(CRAZYSWARM_SCRIPTS_FILE_PATH)
    except FileNotFoundError:
        print("\nERROR: the value of CRAZYSWARM_SCRIPTS_FILE_PATH in Hardware/Hardware_constants.py is not a valid file path. \nSee the comments in the Hardware/Hardware_constants.py or the documentation for how to resolve this.\n")
        return

    print("INITIALISING CRAZYSWARM")
    try:
        swarm = Crazyswarm()
    except FileNotFoundError:
        print("\nERROR: Crazyswarm installation was not found by the path execution code. Check that the value of CRAZYSWARM_SCRIPTS_FILE_PATH in Hardware/Hardware_constants.py is set correctly. \nSee the comments in the Hardware/Hardware_constants.py or the documentation for how to resolve this.\n")
        return
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    next_moves = np.array([]) # Number of seconds to next action change, for each drone

    all_drones, next_moves = parse_input(input_path, allcfs, SPEED, next_moves)

    init_logging()
    log_all_status(all_drones, msg="Initialising logging")
    
    if run == True:
        print("EXECUTING PATH")
        try:
            take_off_all(2.5, timeHelper, all_drones, sequential=False)

            if IN_SIMULATION == False:
                set_initial_positions(timeHelper,all_drones,5)

            follow_plans(timeHelper, all_drones, next_moves)

        except KeyboardInterrupt as error:
            print("Error:",error)
            land_all(0.05, timeHelper, all_drones)

if __name__ == '__main__':
    # [--sim], [path], [input_mode]
    args = sys.argv
    offset = 0
    filepath = None
    input_mode = INPUT_MODE # set both here and in main(...) to allow it to be set from command line or function call to main

    if '--sim' in args:
        IN_SIMULATION = True
        offset = 1

    for arg in args[1+offset:]:
        if arg in ["default", "cdca"]:
            input_mode = arg
        elif os.path.isfile(arg):
            filepath=arg

    #NB: There is no support for passing raw plans (not in a file) through command line arguments, but these can be passed as a parameter when calling main(...) from code with raw=True

    if filepath != None:
        main(filepath,input_mode=input_mode,raw=False)

    else:
        print("ERROR: No path file given")