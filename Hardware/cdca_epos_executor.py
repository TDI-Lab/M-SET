import numpy as np
import os
import sys
import math
# print the original sys.path
print('Original sys.path:', sys.path)

# append a new directory to sys.path
sys.path.append('/home/hritik/crazyswarm/ros_ws/src/crazyswarm/scripts')

# print the updated sys.path
print('Updated sys.path:', sys.path)

from pycrazyswarm import Crazyswarm

#input_path = [[[[0,0],3],[[1,1],6],[[0,1],0]],[[[2,1],3],[[1,0],12],[[2,0],0]]]

# Parameters
input_file_path="example-cdca-output.txt"
crazyswarm_scripts_file_path="/home/hritik/crazyswarm/ros_ws/src/crazyswarm/scripts"
all_drones = []
next_moves = np.array([]) # Number of timeslots to next action, for each drone
global_travel_time = 3 # Only used if travel_time_mode=0
travel_time_mode = 2 # 0=constant duration. 1=constant speed, round up. 2=constant speed, buffer
sensing_time = 1
Z = 1
speed = 0.05 #speed of drone, m/s

# Translate the positions on the testbed to coordinates ((0,0) as the centre of the testbed)
position_to_coords = {
    "[1,1.5]": [0,0,Z], # centre of screen
    "[0,0]": [-0.5533,-0.235,Z],
    "[1,0]": [0, -0.235,Z],
    "[2,0]": [0.5533,-0.235,Z],
    "[0,1]": [-0.5533, 0.235,Z],
    "[1,1]": [0,0.235,Z],
    "[2,1]": [0.5533,0.235,Z],
    "[2.5,1.5]": [0.8299, 0.47,Z] # top right corner of testbed
}

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

class Drone():
    def __init__(self, drone, speed):
        self.positions = []
        self.times = [] 
        self.status = "idle" # idle -> (sensing, waiting, moving)
        self.speed = speed
        self.drone = drone

    def move_next_cell(self):
        self.status = "moving"
        print(i,"moving to",self.positions[0])
        pos = position_to_coords[self.positions[0]]
        if travel_time_mode == 0:
            travel_time = global_travel_time # use the constant travel duration mode
        elif travel_time_mode > 0:
            travel_time = self.calc_travel_time()
        else:
            print("ERROR: Invalid value for travel_time_mode.")
            return -1

        self.drone.goTo(pos, 0, travel_time)
        self.positions.pop(0)

        # Alter the return value if needed
        if travel_time_mode == 1:
            travel_time = math.ceil(travel_time)
        elif travel_time_mode == 2:
            pass

        return (travel_time + 1)
    
    def calc_travel_time(self):
        x_dist = (position_to_coords[self.positions[0]][0]**2) - (self.drone.position()[0]**2)
        y_dist = (position_to_coords[self.positions[0]][1]**2) - (self.drone.position()[1]**2)
        dist = math.sqrt(x_dist**2 + y_dist**2)

        time = dist / self.speed

        return time

input_path = read_cdca_output(input_file_path)

# Change directory to the crazyswarm/scripts folder
# Required to access crazyswarm source files, since Crazyswarm assumes it is being run from a file in the scripts folder
os.chdir(crazyswarm_scripts_file_path)

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs

#parse the input
c = 0
for drone in input_path:
    if c < len(allcfs.crazyflies):
        d = Drone(allcfs.crazyflies[c],speed)
        all_drones.append(d)
        for position in drone:
            d.positions.append(str(position[0]).replace(' ',''))
            d.times.append(int(position[1]))
        
        next_moves = np.append(next_moves, 0) # Queue 0 so that the drone immediately seeks its next action

    c+=1

"""
# Don't think this is actually used now?
# Calculate the maximum time for the simulation
max_time = 0 
for i in range(0,len(input_path)):
    max_time = max(max_time, sum(all_drones[i].times) + (len(input_path)+1)*(travel_time + sensing_time)) 
    # This is wrong, it needs to add sensing and travel time too
"""
    
# Tell the drones to take off
for cf in all_drones:
    cf.drone.takeoff(targetHeight=1.0, duration=2.5)
    timeHelper.sleep(2.5)

# Set the initial positions of the drones in the simulation
# For some reason this only works if it's after the takeoff
for cf in all_drones:
    pos = position_to_coords[cf.positions[0]]
    cf.drone.goTo(pos,0,0)
    cf.positions.pop(0)

    # Originally tried to implement this with cmdPosition(), but had issues switching between low and high level command modes
    #cf.drone.cmdPosition(pos)
    #cf.drone.notifySetpointsStop(0)

# Cycle through the time slots
# If a drone moves at that time slot, move it
t=0 # timeslot counter
while np.any(next_moves > -1): # need to change this to >= 0 if allow timeslots of size <1 sec

    print("t=",t)

    if travel_time_mode == 2:
        if np.any((next_moves < 1) & (next_moves > 0)):
            for j in range(0,len(next_moves)):
                if (next_moves[j] < 1 and next_moves[j] > 0):
                    # round up the travel_time of the subject drone
                    next_moves[j] = math.ceil(next_moves[j])
                else:
                    # add one to the action time of the other drones
                    next_moves[j] = next_moves[j] + 1

    for i in range(0,len(all_drones)):

        cf = all_drones[i]
        if next_moves[i] == 0: # if it's time for the drone to change status (i.e. it has finished its current task)

            # Could implement a move counter in Drone class e.g. drone.nmoves to track how many cells it had visited, then do drone.positions[nmoves] and times[nmoves] instead of costly(?) pop() operations
            # Sensing and waiting could technically be combined into one state of length (sensing_time + wait time) since they both currently just involve staying in the same spot, but have split them here to allow extension for different functionality in each state (e.g. to perform sensing actions) 

            if cf.status == "moving" or cf.status == "idle":
                cf.status = "sensing"
                print(i, "sensing")
                next_moves[i] = sensing_time +1

            elif cf.status == "sensing":
                cf.status = "waiting"
                print(i,"waiting for",cf.times[0])
                next_moves[i] = cf.times[0] +1
                cf.times.pop(0)

                # if that was the last position, mark the drone as finished
                # NB: This assumes there is no wait time at the last cell in the drones path
                if len(cf.times) < 1:  
                    # if all times are used up, then mark drone as done
                    next_moves[i] = -1

                # otherwise, we can consider...
                # if wait time was 0 then need to move straight to moving in this iteration too
                elif next_moves[i] == 0:
                    next_moves[i] = cf.move_next_cell()

            else: # cf.status == "waiting"
                next_moves[i] = cf.move_next_cell()

    # minus 1 second from all next_moves (represents 1 second passing)
    next_moves = next_moves - np.full((1,len(all_drones)),1)[0]

    # increment timeslot
    timeHelper.sleep(2)
    t+=1

# Give some extra time so that the simulation doesn't shut down abruptly as soon as the drones stop moving
timeHelper.sleep(3)

# Time does not seem to naturally pass in this script
# This can be verified by calling TimeHelper.time(), which returns the current time, and seeing that this only increases when a call to timeHelper.sleep() is made
# Otherwise the time stays at its current value, so if no calls to timeHelper.sleep are made then it stays at 0 the whole simulation
