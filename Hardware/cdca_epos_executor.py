import numpy as np
import os
import sys
import math
from decimal import Decimal
import rospy
from std_msgs.msg import String

try:
    from Hardware_constants import *
except:
    from Hardware.Hardware_constants import * 
    
#from aSync import aSync
from ROSListener import *

# append a new directory to sys.path
sys.path.append(CRAZYSWARM_SCRIPTS_FILE_PATH)
from pycrazyswarm import Crazyswarm
Z=HOVER_HEIGHT

pub = rospy.Publisher('status_logger', String, queue_size=10)
#rospy.init_node('status', anonymous=True)
#rate = rospy.Rate(10) # 10hz

def get_coords(position, use_cell_coords):
    if use_cell_coords == True:
        #return default_epos_coords[str(position).replace(' ','')]
        x = convert_coords(position[0],"x")
        y = convert_coords(position[1],"y")
        z = HOVER_HEIGHT # should be HOVER_HEIGHT, but hardcoding it for now to narrow down possible errors
        return([x,y,z])
    else:
        return [position[0],position[1],HOVER_HEIGHT]

def convert_coords(val, axis):
    if axis == "x":
        return (val-2)*0.553
    elif axis == "y":
        return ((2*val)-3)*0.235
    else:
        return NameError

"""
# Translate the positions on the testbed to coordinates ((0,0) as the centre of the testbed)
default_epos_coords = {
    "(1.0,1.0,1.0)": [-0.5533,-0.235,HOVER_HEIGHT], # cell 0
    "(2.0,1.0,1.0)": [0, -0.235,HOVER_HEIGHT], # cell 1
    "(3.0,1.0,1.0)": [0.5533,-0.235,HOVER_HEIGHT], # cell 2
    "(1.0,2.0,1.0)": [-0.5533, 0.235,HOVER_HEIGHT], # cell 3
    "(2.0,2.0,1.0)": [0,0.235,HOVER_HEIGHT], # cell 4
    "(3.0,2.0,1.0)": [0.5533,0.235,HOVER_HEIGHT], # cell 5
    "(0.0,0.0,0.0)": [-0.8299,-0.47,0.5], # Base 0, bottom left corner
    "(4.0,0.0,0.0)": [0.8299,-0.47,HOVER_HEIGHT], # Base 1, bottom right corner
    "(4.0,3.0,0.0)": [0.8299, 0.47,HOVER_HEIGHT], # Base 2, top right corner
    "(0.0,3.0,0.0)": [-0.8299, 0.47,HOVER_HEIGHT] # Base 3, top left corner
}
"""

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

def round_nearest(number, base): # Round to the nearest 'base' e.g. nearest 5
    return base * round(number/base)

def roundup_nearest(number, base): # Round UP to the nearest 'base' e.g. nearest 5
    # if remainder of number/base is 0 (base divides number exactly), then don't round up further
    #   e.g. roundup_nearest(25,5) = 5, NOT 6

    diff = (number/base) - round(number/base) # diff>0 if rounded down, diff<0 if rounded up, diff=0 if exact division

    if diff > 0:
        return max(base * round(number/base), base)
    else:
        return max(base * (round(number/base) + 1), base)

class Drone():
    def __init__(self, cf, speed):
        self.positions = []
        self.times = [] 
        self.status = "idle" # idle -> hovering -> ([sensing or waiting] moving)
        self.speed = speed
        self.cf = cf
        self.id = self.cf.id # DEPRECATED
        self.move_count = 0 # Count of moves completed by the drone

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
        #self.positions.pop(0)

        # Alter the return value if needed
        if TRAVEL_TIME_MODE == 1:
            travel_time = roundup_nearest(travel_time, TIMESTEP_LENGTH)
        elif TRAVEL_TIME_MODE == 2:
            pass
        elif TRAVEL_TIME_MODE == 3:
            travel_time = roundup_nearest(travel_time, TIMESTEP_LENGTH)

        return (travel_time + TIMESTEP_LENGTH)
    
    def calc_travel_time(self):
        x_dist = (get_coords(self.positions[self.move_count], USE_CELL_COORDS)[0]) - (get_coords(self.positions[self.move_count-1], USE_CELL_COORDS)[0])
        y_dist = (get_coords(self.positions[self.move_count], USE_CELL_COORDS)[1]) - (get_coords(self.positions[self.move_count-1], USE_CELL_COORDS)[1])

        dist = math.sqrt((x_dist**2 + y_dist**2))

        time = dist / self.speed

        print(x_dist, y_dist, dist, self.speed, time)

        return time
    
    def land_drone(self, timeHelper):
        # Land the drones
        # NB: This action is performed differently in simulation vs. on real hardware 
        if IN_SIMULATION == False:
            # Calling the land command, even on just one drone, makes all of them disappear from the simulation view
            #   Therefore, this can only be done when not in simulation
            self.cf.land(0.05, 2.5)
        else: 
            # This is a workaround to avoid calling land command. The code below performs same functionality as the land command in this instance (but does so in the simulation)
            land_pos = get_coords(self.positions[self.move_count-1],USE_CELL_COORDS)
            self.cf.goTo((land_pos[0],land_pos[1],0.05),0,2.5)
        timeHelper.sleep(2.5)

    def log_status(self, msg=""):
        #rospy.loginfo(self.status)
        if msg != "" and PRINT_LOG_MESSAGES==True:
            print(msg)
        
        if ENABLE_LOGGING == True:
            pub.publish("Drone %s: %s. %s" % self.id, self.status, msg)

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
                    d.times.append(int(position[1]))
                elif INPUT_MODE == "default":
                    d.positions.append(position)
                    d.times.append(0)
            
            next_moves = np.append(next_moves, 0) # Queue 0 so that the drone immediately seeks its next action

        c+=1

    return all_drones, next_moves

def take_off_all(dur, timeHelper, all_drones, all_cfs=None, sequential=False):
    if sequential == True:
        if all_cfs == None:
            all_cfs = [drone.cf for drone in all_drones]
        all_cfs.takeoff(targetHeight=HOVER_HEIGHT, duration=dur)
        timeHelper.sleep(2.5)
        log_all_drones(all_drones)
    else:
        # Tell the drones to take off one at a time
        for drone in all_drones:
            drone.cf.takeoff(targetHeight=HOVER_HEIGHT, duration=dur)
            timeHelper.sleep(2.5)
            drone.cf.status = "hovering"
            drone.cf.log_status(msg="Drone %s taken off" % drone.cf.id)

def land_all(d, timeHelper,all_drones):
# Tell the drones to take off
    for drone in all_drones:
        drone.cf.land(0.05, 2.5)
        timeHelper.sleep(2.5)

def set_initial_positions(timeHelper, all_drones, duration):
    # Set the initial positions of the drones in the simulation
    # For some reason this only works if it's after the takeoff
    for drone in all_drones:
        pos = get_coords(drone.positions[drone.move_count], USE_CELL_COORDS)
        drone.status="moving"
        drone.cf.log_status("Drone %s moving to %s" % (drone.cf.id, pos))
        drone.cf.goTo(pos,0,duration)
        timeHelper.sleep(duration)
        drone.status="hovering"
        drone.cf.log_status("Drone %s reached initial position %s" % (drone.cf.id, pos))

def return_uris(channels,numbers):
    uris = []
    for i in range(0,len(channels)):
        uris.append("radio://0/"+str(channels[i])+"/2M/E7E7E7E7"+"0"+str(numbers[i])) # Note: the 0 only needs to be there for drone IDs < 10 - need to change this
    return uris

def log_all_status(all_drones,msg=""):
    for drone in all_drones:
        drone.cf.log_status(msg=msg)

def log_all_drones(ids, vars):
    if IN_SIMULATION == False:
        if ENABLE_LOGGING == True:
            call_once(ids)
            #listener(ids)
    """
            logger = aSync(drone_uris)
            logger.runCallback()
    """
    pass
        
def adjust_moves(next_moves):
    if TRAVEL_TIME_MODE == 2:
        if np.any((next_moves < TIMESTEP_LENGTH) & (next_moves > 0)):
            print("Adjusting moves")
            for j in range(0,len(next_moves)):
                if (next_moves[j] < TIMESTEP_LENGTH and next_moves[j] > 0):
                    print("if",j)
                    # round up the travel_time of the subject drone
                    next_moves[j] = roundup_nearest(next_moves[j],TIMESTEP_LENGTH) # remove this to preserve consistent speed? Or does it need to go slower to give other drones the chance to get out of the way?
                else:
                    print("else",j)
                    # add one to the action time of the other drones
                    next_moves[j] = next_moves[j] + TIMESTEP_LENGTH # not sure if this is right or it should be the below instead
                    #next_moves[j] = roundup_nearest(next_moves[i],TIMESTEP_LENGTH) # seems to break it

    return next_moves

def follow_plans(timeHelper, all_drones, next_moves):
    log_all_status(all_drones, msg="Starting to follow plans")

    # Cycle through the time slots
    # If a drone moves at that time slot, move it
    t=0 # timeslot counter
    while np.any(next_moves > -1): # need to change this to >= 0 if allow timeslots of size >1 sec?

        if t % 1 == 0: # Only print integer values of t
            print("t=",t)

        # Adjust next_moves to account for latency between timesteps
        #next_moves = adjust_moves(next_moves, TIMESTEP_LENGTH, TRAVEL_TIME_MODE) # REMOVE THIS?

        in_position = all(drone.status == "idle" for drone in all_drones)
        for i in range(0,len(all_drones)):

            cf = all_drones[i]

            # if it's time for the drone to change status (i.e. it has finished its current task)
            if round_nearest(next_moves[i], TIMESTEP_LENGTH) == 0: # IS THIS ROUNDING CORRECT?  

                # LOG FINISHED ACTION
                cf.log_status()
                #log_all_drones([1],["battery"])

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
                    #log_all_drones([1],["battery"])

                # OTHERWISE, CONTINUE
                else:

                    # ASSIGN STATUS CHANGES
                    if cf.status == "moving" or cf.status == "idle":
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
                            print(i, "idle")
                            next_moves[i] = TIMESTEP_LENGTH
                    
                    elif cf.status == "waiting" or cf.status == "sensing":
                        cf.status = "moving"

                    #print(cf.status)

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
        timeHelper.sleep(TIMESTEP_LENGTH) # Replace with timeHelper.sleepForTime(Hz)?
        t = round_nearest(t + TIMESTEP_LENGTH, TIMESTEP_LENGTH)

    # Give some extra time so that the simulation doesn't shut down abruptly as soon as the drones stop moving
    cf.log_all_status(all_drones, msg="End of simulation")
    timeHelper.sleep(3)


"""
Parameters:
IN_SIMULATION - Is it being run in simulation - True or False
INPUT_MODE - "default": epos with no cdca, "cdca": waiting cdca
input_file_path - path to input file
"""
def main(plan, raw=False, travel_time_mode=2, use_cell_coords=True, sensing_time=0, timestep_length=1, global_travel_time=6, run=True, input_mode=INPUT_MODE):
    # Allows INPUT_MODE to be overwritten by supplying a new value (as opposed to using the one from the config file)
    global INPUT_MODE
    INPUT_MODE = input_mode

    print("PARSING INPUT")
    if raw == True:
        input_path = plan
    else:
        if INPUT_MODE == "cdca":
            input_path = read_cdca_output(plan)
        elif INPUT_MODE == "default":
            input_path = read_default_output(plan)
    print("Path=",input_path)

    # Change directory to the crazyswarm/scripts folder
    # Required to access crazyswarm source files, since Crazyswarm assumes it is being run from a file in the scripts folder
    os.chdir(CRAZYSWARM_SCRIPTS_FILE_PATH)

    print("INITIALISING CRAZYSWARM")
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    next_moves = np.array([]) # Number of timeslots to next action, for each drone

    all_drones, next_moves = parse_input(input_path, allcfs, SPEED, next_moves)

    #drone_uris = return_uris([80,90],[2,3])

    print("INITIALISING LOGGING")
    log_all_status(all_drones, msg="Initialising logging")
    """
    ids = [1,2]
    log_all_drones(ids, ["battery"])
    """
    
    if run == True:
        print("EXECUTING PATH")
        try:
            take_off_all(2.5, timeHelper, all_drones, sequential=False)

            set_initial_positions(timeHelper,all_drones)

            follow_plans(timeHelper, all_drones, next_moves)

        except Exception as error:
            print("Error:",error)
            land_all(0.05, timeHelper, all_drones)

    #land_all(0.05, timeHelper, all_drones)

    #log_all_drones(drone_uris, ["battery"])

if __name__ == '__main__':
    # [--sim], [path], [input_mode]
    
    args = sys.argv
    offset = 0
    filepath = None
    input_mode = INPUT_MODE # set both here and in main(...) to allow it to be set from command line or function call to main

    if '--sim' in args:
        IN_SIMULATION = True
        offset = 1

    #if len(args) > 1+offset: # if any arguments given (excluding '--sim')
    for arg in args[1+offset:]:
        if arg in ["default", "cdca"]:
            input_mode = arg
        elif os.path.isfile(arg):
            filepath=arg

    #NB: There is no support for passing raw plans (not in a file) through command line arguments, but these can be passed as a parameter when calling main(...) from code with raw=True

    if filepath != None:
        main(filepath,input_mode=input_mode,raw=False)

    else: # Here for debugging purposes
        
        #main("epospaths/Evangelos_cdca_demo4.txt",run=True)
        #main("epospaths/April/debug_default_4_fake.txt", input_mode="default")
        #main("epospaths/April/debug_cdca_4_fake.txt", input_mode="cdca")
        #main("Hardware/epospaths/April/16cells.txt", input_mode="default", raw=False)
        main("epospaths/April/16cells.txt", input_mode="default", raw=False, run=True)

# Debugging demos    
#main(True, "default", "epospaths/debug_default_demo.txt", 2, True, 1, 0.5, 0.1)
#main(True, "cdca", "epospaths/debug_cdca_demo.txt", 2, True, 1, 0.5, 0.1)

# Demos
#main(True, "default", "epospaths/Evangelos_default_demo.txt", 2, True, 1, 0.5, 0.1)
#   main(True, "cdca", "epospaths/Evangelos_cdca_demo4.txt", 2, True, 0, 0.5, 0.1, 0.5)
#main(True, "default", "epospaths/sanity_test.txt", 2, True, 1, 0.5, 0.1)

# Collision demos
#main(True, "default", "epospaths/head_on_demo.txt", 2, True, 1, 0.5, 0.1)
#main(True, "cdca", "epospaths/head_on_cdca_demo.txt", 2, True, 1, 0.5, 0.1)
#main(True, "default", "epospaths/cross_demo.txt", 2, True, 1, 0.5, 0.1)
#main(True, "cdca", "epospaths/head_on_cdca_demo.txt", 2, True, 1, 0.5, 0.1)
        
#log_all_drones(return_uris([80],[1]), ["battery"])
#log_all_drones(return_uris([80,90],[2,3]), ["battery"])