import sys

# Adds the directory above the one containing this script (where the pycrazyswarm module is stored) to the path list
sys.path.append('/'.join(sys.path[0].split('/')[0:len(sys.path[0].split('/'))-1]))
from pycrazyswarm import *

import numpy as np

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs

current_position = np.zeros((len(allcfs.crazyflies),3))

# Translate the positions on the testbed to coordinates ((0,0) as the centre of the testbed)
position_to_coords = {
    -1: [0,0], # centre of screen
    0: [-0.5533,-0.235],
    1: [0, -0.235],
    2: [0.5533,-0.235],
    3: [-0.5533, 0.235],
    4: [0,0.235],
    5: [0.5533,0.235],
    6: [0.8299, 0.47] # top right corner of testbed
}

def read_epos_output(filename):
    try:
        f = open(filename,"r")
    except FileNotFoundError:
        print("Error: FileNotFoundError")
        return []
    except:
        print("Error: Something unknown went wrong when attempting to access the file (not FileNotFoundError)")
        return []

    epos_output = f.read()
    positions = []
    for index, line in enumerate(epos_output.splitlines()):
        if index != 0: # skip the first line which is just the column headings
            positions.append(line.split(',')[1:])
            positions[index-1].pop()# included to remove the ,'' element at the end of lines
    print("Paths = ",positions)

    return positions

def takeoff(Z, z_step):
    # takeoff
    while Z < 1.0:
        for agentID in range(0,len(allcfs.crazyflies)):
            cf = allcfs.crazyflies[agentID]
            pos = current_position[agentID] + np.array([0, 0, z_step])
            cf.cmdPosition(pos)
            current_position[agentID] = pos

        timeHelper.sleep(0.1)
        Z += 0.05
        
def set_initial_positions(positions, sim):
    timeHelper.sleep(5)
    # record the initial positions of the crazyflie drones
    for agentID in range(0,len(allcfs.crazyflies)):
        if sim == False:
            # take the actual current position of the drones
            current_position[agentID] = [allcfs.crazyflies[agentID].initialPosition[0], allcfs.crazyflies[agentID].initialPosition[1], allcfs.crazyflies[agentID].initialPosition[2]]
        else:
            # set the drones' recorded position to their starting positions from the epos path
            current_position[agentID] = [position_to_coords[int(positions[0][agentID])][0], position_to_coords[int(positions[0][agentID])][1], 0]
            
            # set the drones' position in the simulation to their starting positions from the epos path
            allcfs.crazyflies[agentID].cmdPosition(current_position[agentID])

    timeHelper.sleep(5)

def follow_epos(positions, hover_height, x_step, y_step):
    # follow EPOS
    for t in range(1,len(positions)): # Starts at 1 since the drones should already be at the first position
        print("Moving to position: ", t)

        # set the x and y directions that each drone will move towards its target position
        # move[agentID] = [x direction \in {-1,0,1}, y direction \in {-1,0,1}]
        move = np.zeros((len(allcfs.crazyflies),3))
        for agentID in range(0,len(allcfs.crazyflies)):
            cf = allcfs.crazyflies[agentID]
            target_x = position_to_coords[int(positions[t][agentID])][0]
            target_y = position_to_coords[int(positions[t][agentID])][1]
            target_Z = hover_height
            target_position = np.array([target_x, target_y, target_Z])
            
            # floating point error here, causes z difference to be -1 even if equal
            difference = target_position - current_position[agentID] # calculate distance vector to target position
            move[agentID] = np.divide(difference,np.absolute(difference), out=np.zeros_like(np.absolute(difference)), where=difference!=0) # normalise to have a magnitude of 1. 'where' parameter is used to account for divide by 0 here.

        in_position = np.zeros((len(allcfs.crazyflies),2))

        # set the speeds of the drones (relative to the step distances)
        # could be used to implement pausing or slowing of chosen drones to support collision avoidance
        speed = np.full((len(allcfs.crazyflies),2),1)

        while (np.any(in_position == 0)): # while there exists at least one drone that is out of position
            for agentID in range(0,len(allcfs.crazyflies)): # will only run the paths for as many drones as are available (if there are paths for more drones than are active, then these paths won't be used)
                cf = allcfs.crazyflies[agentID]
                target_x = position_to_coords[int(positions[t][agentID])][0]
                target_y = position_to_coords[int(positions[t][agentID])][1]

                # move at constant speed towards target (x,y) coord
                #If the drone is not at its target x position
                if (move[agentID][0]*(current_position[agentID][0]) < move[agentID][0]*(target_x)):
                    pos = current_position[agentID] + np.array([move[agentID][0]*speed[agentID][0]*x_step, 0, 0])
                    cf.cmdPosition(pos)
                    current_position[agentID] = pos
                else:
                    in_position[agentID][0] = 1

                #If the drone is not at its target y position
                if (move[agentID][1]*(current_position[agentID][1]) < move[agentID][1]*(target_y)):
                    pos = current_position[agentID] + np.array([0, move[agentID][1]*speed[agentID][1]*y_step, 0])
                    cf.cmdPosition(pos)
                    current_position[agentID] = pos
                else:
                    in_position[agentID][1] = 1

            timeHelper.sleep(0.1) # refresh rate of giving setpoint commands. Documentation suggests this shouldn't be any lower that 10Hz    

        # Carry out sensing task of one epos timestep (or rather, pause the drones for one epos timestep to allow them to do this)
        timeHelper.sleep(10) # needs to be replaced with sleep(timestep of epos)

def land(Z, z_step):
    # land
    while Z > 0.01:
        for agentID in range(0,len(allcfs.crazyflies)):
            cf = allcfs.crazyflies[agentID]
            pos = current_position[agentID] + np.array([0, 0, -z_step])
            cf.cmdPosition(pos)
            current_position[agentID] = pos

        timeHelper.sleep(0.1)
        Z -= 0.05

    # turn-off motors
    for cf in allcfs.crazyflies:
        cf.cmdStop()

def main():
    args = sys.argv

    print("args: ", args)

    for arg in args:
        if arg == "--sim":
            sim = True
            args.remove(arg)

    if len(args) == 2:
        filename = sys.argv[1]
    elif len(args) == 1:
        print ("Error: Too few command line arguments - please include the path to a csv file containing the epos paths as the first command line argument.\nExiting")
        return
    elif len(args) > 2:
        print ("Error: Too many command line arguments.\nExiting")
        return
    
    hover_height = 1
    x_step = 0.05 # how far the drone moves at each increment in the x direction
    y_step = 0.05 # how far the drone moves at each increment in the y direction
    z_step = 0.05 # how far the drone moves at each increment in the z direction

    print ("Correct number of command line arguments recieved")
    print("starting")
    positions = read_epos_output(args[1])

    if len(positions) > 0:
        set_initial_positions(positions, sim)
        takeoff(allcfs.crazyflies[0].initialPosition[2], z_step)
        timeHelper.sleep(3)
        follow_epos(positions, hover_height, x_step, y_step)
        land(hover_height, z_step)
    
    print("end")

if __name__ == "__main__":
    main()