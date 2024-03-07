import numpy as np
import os
import sys
import math

#crazyswarm_scripts_file_path="/path/to/crazyswarm/scripts"
crazyswarm_scripts_file_path = "/home/adam/Documents/Packages/crazyswarm/ros_ws/src/crazyswarm/scripts"
# append a new directory to sys.path
sys.path.append(crazyswarm_scripts_file_path)
from pycrazyswarm import Crazyswarm
Z=0.5

def get_coords(position, use_cell_coords, input_mode):
    if use_cell_coords == True:
        if input_mode == "default":
            #return default_epos_coords[str(position).replace(' ','')]
            x = convert_coords(position[0],"x")
            y = convert_coords(position[1],"y")
            z = Z
            return([x,y,z])
        elif input_mode == "cdca":
            return epos_cdca_coords[str(position).replace(' ','')]
    else:
        return [position[0],position[1],Z]

def convert_coords(val, axis):
    if axis == "x":
        return (val-2)*0.553
    elif axis == "y":
        return (val-2)*0.235
    else:
        return NameError

# Translate the positions on the testbed to coordinates ((0,0) as the centre of the testbed)
cell_coords = {
    "[1.0,1.5]": [0,0,Z], # centre of screen
    "[0.0,0.0]": [-0.5533,-0.235,Z],
    "[1.0,0.0]": [0, -0.235,Z],
    "[2.0,0.0]": [0.5533,-0.235,Z],
    "[0.0,1.0]": [-0.5533, 0.235,Z],
    "[1.0,1.0]": [0,0.235,Z],
    "[2.0,1.0]": [0.5533,0.235,Z],
    "[2.5,1.5]": [0.8299, 0.47,Z] # top right corner of testbed
}

epos_cdca_coords = {
    "[0.0,0.0]": [0.0,0.0,Z],
    "[1.0,1.0]": [-0.5533,-0.235,Z], #0
    "[2.0,1.0]": [0, -0.235,Z], #1
    "[3.0,1.0]": [0.5533,-0.235,Z], #2
    "[1.0,2.0]": [-0.5533, 0.235,Z], #3
    "[2.0,2.0]": [0,0.235,Z], #4
    "[3.0,2.0]": [0.5533,0.235,Z], #5
    "[4.0,0.0]": [0.8299, 0.47,Z]
}

default_epos_coords = {
    "(1.0,1.0,1.0)": [-0.5533,-0.235,Z], #0
    "(2.0,1.0,1.0)": [0, -0.235,Z], #1
    "(3.0,1.0,1.0)": [0.5533,-0.235,Z], #2
    "(1.0,2.0,1.0)": [-0.5533, 0.235,Z], #3
    "(2.0,2.0,1.0)": [0,0.235,Z], #4
    "(3.0,2.0,1.0)": [0.5533,0.235,Z], #5
    "(0.0,0.0,0.0)": [-0.8299,-0.47,Z], # Base 0, bottom left corner
    "(4.0,0.0,0.0)": [0.8299,-0.47,Z], # Base 1, bottom right corner
    "(4.0,3.0,0.0)": [0.8299, 0.47,Z], # Base 2, top right corner
    "(0.0,3.0,0.0)": [-0.8299, 0.47,Z] # Base 3, top left corner

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

class Drone():
    def __init__(self, drone, speed):
        self.positions = []
        self.times = [] 
        self.status = "idle" # idle -> (sensing, waiting, moving)
        self.speed = speed
        self.drone = drone

    def move_next_cell(self, use_cell_coords, travel_time_mode, global_travel_time, i,input_mode):
        self.status = "moving"
        print(i,"moving to",self.positions[0])
        pos = get_coords(self.positions[0],use_cell_coords,input_mode)
        if travel_time_mode == 0:
            travel_time = global_travel_time # use the constant travel duration mode
        elif travel_time_mode > 0:
            travel_time = self.calc_travel_time(use_cell_coords,input_mode)
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
    
    def calc_travel_time(self, use_cell_coords,input_mode):
        print("calc time")
        #x_dist = (get_coords(self.positions[0], use_cell_coords,input_mode)[0]**2) - (self.drone.position()[0]**2)
        #y_dist = (get_coords(self.positions[0], use_cell_coords,input_mode)[1]**2) - (self.drone.position()[1]**2)
        x_dist = (get_coords(self.positions[0], use_cell_coords,input_mode)[0]) - (self.drone.position()[0])
        y_dist = (get_coords(self.positions[0], use_cell_coords,input_mode)[1]) - (self.drone.position()[1])

        #self.drone.position() isn't something you can call on physical hardware

        dist = math.sqrt((x_dist**2 + y_dist**2))

        print(x_dist, y_dist, dist, self.speed)

        time = dist / self.speed
        #time = round(time,2)
        print("duration: "+str(time))

        return 6

def parse_input(input_path, allcfs, input_mode, speed, next_moves):
    all_drones = []

    #parse the input
    c = 0
    for drone in input_path:
        if c < len(allcfs.crazyflies):
            d = Drone(allcfs.crazyflies[c],speed)
            all_drones.append(d)
            for position in drone:
                if input_mode == "cdca":
                    d.positions.append(position[0])
                    d.times.append(int(position[1]))
                elif input_mode == "default":
                    d.positions.append(position)
                    d.times.append(0)
            
            next_moves = np.append(next_moves, 0) # Queue 0 so that the drone immediately seeks its next action

        c+=1

    return all_drones, next_moves

def take_off_all(Z, d, timeHelper,all_drones):
# Tell the drones to take off
    for cf in all_drones:
        cf.drone.takeoff(targetHeight=Z, duration=d)
        timeHelper.sleep(2.5)

def land_all(Z, d, timeHelper,all_drones):
# Tell the drones to take off
    for cf in all_drones:
        cf.drone.land(0.05, 2.5)
        timeHelper.sleep(2.5)

def set_initial_positions(all_drones, use_cell_coords,input_mode):
    # Set the initial positions of the drones in the simulation
    # For some reason this only works if it's after the takeoff
    for cf in all_drones:
        pos = get_coords(cf.positions[0], use_cell_coords,input_mode)
        cf.drone.goTo(pos,0,0)
        cf.positions.pop(0)

def follow_plans(timeHelper, all_drones, next_moves, travel_time_mode, use_cell_coords, sensing_time, global_travel_time,input_mode):
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
                        next_moves[i] = cf.move_next_cell(use_cell_coords, travel_time_mode, global_travel_time, i,input_mode)

                else: # cf.status == "waiting"
                    next_moves[i] = cf.move_next_cell(use_cell_coords, travel_time_mode, global_travel_time, i, input_mode)

        # minus 1 second from all next_moves (represents 1 second passing)
        next_moves = next_moves - np.full((1,len(all_drones)),1)[0]

        # increment timeslot
        timeHelper.sleep(1)
        t+=1

    # Give some extra time so that the simulation doesn't shut down abruptly as soon as the drones stop moving
    timeHelper.sleep(3)


"""
Parameters:
simulation - Is it being run in simulation - True or False
input_mode - "default": epos with no cdca, "cdca": waiting cdca
input_file_path - path to input file
"""
def main(simulation, input_mode, input_file_path, travel_time_mode, use_cell_coords, sensing_time, Z, speed, global_travel_time=6):
    if input_mode == "cdca":
        input_path = read_cdca_output(input_file_path)
    elif input_mode == "default":
        input_path = read_default_output(input_file_path)
    print("Path=",input_path)

    # Change directory to the crazyswarm/scripts folder
    # Required to access crazyswarm source files, since Crazyswarm assumes it is being run from a file in the scripts folder
    os.chdir(crazyswarm_scripts_file_path)

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    next_moves = np.array([]) # Number of timeslots to next action, for each drone

    all_drones, next_moves = parse_input(input_path, allcfs, input_mode, speed, next_moves)

    take_off_all(Z, 2.5, timeHelper, all_drones)

    if simulation == True:
        set_initial_positions(all_drones, use_cell_coords,input_mode)

    try:
        pass
        follow_plans(timeHelper, all_drones, next_moves, travel_time_mode, use_cell_coords, sensing_time, global_travel_time,input_mode)
    except Exception as error:
        print(error)
        land_all(Z, 0.05, timeHelper, all_drones)  

    land_all(Z, 0.05, timeHelper, all_drones)
    
#main(True, "cdca", "example_cdca_output.txt", 2, True, 1, 1, 0.05)
#main(True, "cdca", "example_cdca_output.txt", 2, True, 2, 0.5, 0.05)

#main(True, "cdca", "epospaths/cdca_demo3.txt", 2, True, 2, 0.5, 0.05)
main(False, "default", "epospaths/default_demo.txt", 1, True, 0, 0.5, 0.05)

#print(read_default_output("epospaths/default_demo.txt")[0][0][0])