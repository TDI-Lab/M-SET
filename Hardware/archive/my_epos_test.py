import sys

# Adds the directory above the one containing this script (where the pycrazyswarm module is stored) to the path list
sys.path.append('/'.join(sys.path[0].split('/')[0:len(sys.path[0].split('/'))-1]))
from pycrazyswarm import *

import numpy as np

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs

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
            positions[index-1].pop()# included to remove the ,'' element at the end of lines, but also removes the 'agent-10' item in the first line 
    print(positions)

    return positions

# Translate the positions on the screen to coordinates ((0,0) as the centre of the screen)
position_to_coords = {
    -1: [0,0],
    0: [-0.5533,-0.235],
    1: [0, -0.235],
    2: [0.5533,-0.235],
    3: [-0.5533, 0.235],
    4: [0,0,235],
    5: [0.5533,0.235],
    6: [0.94,1.66]
}

def takeoff(Z):
    # takeoff
    while Z < 1.0:
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
            cf.cmdPosition(pos)
        timeHelper.sleep(0.1)
        Z += 0.05

def follow_epos(positions):
    # follow EPOS
    for t in range(0,len(positions)):
        for agentID in range(0,len(allcfs.crazyflies)): # will only run the paths for as many drones as are available (if there are paths for more drones than are active, then these paths won't be used)
            cf = allcfs.crazyflies[agentID]
            x = position_to_coords[int(positions[t][agentID])][0]
            y = position_to_coords[int(positions[t][agentID])][1]
            Z = cf.initialPosition[2]
            target_position = np.array([x, y, Z])
            cf.cmdPosition(target_position)
            timeHelper.sleep(1)
        timeHelper.sleep(10)

def land(Z):
    # land
    while Z > 0.01:
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
            cf.cmdPosition(pos)
        timeHelper.sleep(0.1)
        Z -= 0.05

    # turn-off motors
    for cf in allcfs.crazyflies:
        cf.cmdStop()

def main():
    args = sys.argv

    print("args: ", args)

    for arg in args:
        if "--" in arg:
            args.remove(arg)

    if len(args) == 2:
        filename = sys.argv[1]
    elif len(args) == 1:
        print ("Error: Too few command line arguments - please include the path to a csv file containing the epos paths as the first command line argument.\nExiting")
        return
    elif len(args) > 2:
        print ("Error: Too many command line arguments.\nExiting")
        return
    
    print ("Correct number of command line arguments recieved")
    print("starting")
    #positions = read_epos_output("my_epos_test_input.csv")
    #positions = read_epos_output("epos_basic_test.csv")
    positions = read_epos_output(args[1])

    if len(positions) > 0:
        takeoff(0)
        follow_epos(positions)
        land(1)

    print("end")

if __name__ == "__main__":
    main()