import numpy as np

# Add chdir here to change to directory containing crazyswarm installation

from pycrazyswarm import Crazyswarm

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs

input = [[[[0,0],3],[[1,1],6]],[[[2,1],9],[[1,0],12]]]

# Translate the positions on the testbed to coordinates ((0,0) as the centre of the testbed)
position_to_coords = {
    "[1,1.5]": [0,0], # centre of screen
    "[0,0]": [-0.5533,-0.235],
    "[1,0]": [0, -0.235],
    "[2,0]": [0.5533,-0.235],
    "[0,1]": [-0.5533, 0.235],
    "[1,1]": [0,0.235],
    "[2,1]": [0.5533,0.235],
    "[2.5,1.5]": [0.8299, 0.47] # top right corner of testbed
}

class Drone():
    def __init__(self, drone):
        self.positions = []
        self.times = [] 
        self.status = "idle" # idle -> (sensing, waiting, moving)
        self.drone = drone

# Parameters
all_drones = []
next_moves = np.array([]) # Number of timeslots to next action, for each drone
travel_time = 3
sensing_time = 1

#parse the input
c = 0
for drone in input:
    d = Drone(allcfs.crazyflies[0])
    all_drones.append(d)
    for position in drone:
        d.positions.append(str(position[0]).replace(' ',''))
        d.times.append(int(position[1]))
    
    next_moves = np.append(next_moves, 0) # Queue the time that the drone waits at the first position

    c+=1

max_time = 0 
for i in range(0,len(input)):
    max_time = max(max_time, sum(all_drones[i].times) + (len(input)+1)*(travel_time + sensing_time)) 
    # This is wrong, it needs to add sensing and travel time too

cf = swarm.allcfs.crazyflies[0]
cf.takeoff(targetHeight=1.0, duration=2.5)
timeHelper.sleep(7.5)

# Cycle through the time slots
# If a drone moves at that time slot, move it
t=0
while np.any(next_moves > -1):

    print("t=",t)

    for i in range(0,len(all_drones)):

        cf = all_drones[i]
        if next_moves[i] == 0: # if it's time for the drone to change status (i.e. it has finished its current task)

            # Could implement a move counter in Drone class e.g. drone.nmoves to track how many cells it had visited, then do drone.positions[nmoves] and times[nmoves] instead of costly(?) pop() operations
            # Sensing and waiting could technically be combined into one state of length (sensing_time + wait time) since they both currently just involve staying in the same spot, but have split them here to allow extension for different functionality in each state (e.g. to perform sensing actions) 

            if cf.status == "idle":
                cf.status = "sensing"
                next_moves[i] = sensing_time +1

            elif cf.status == "moving":
                cf.status = "sensing"
                next_moves[i] = sensing_time +1

            elif cf.status == "sensing":
                cf.status = "waiting"
                next_moves[i] = cf.times[0] +1
                cf.times.pop(0)

                """
                # if wait time was 0 then need to move straight to moving in this iteration too
                if next_moves[i] == 0:
                    # Literally just copy-pasted from below
                    cf.status = "moving"
                    cf.drone.goTo(position_to_coords[cf.positions[0]], 0, travel_time)
                    cf.positions.pop(0)
                    next_moves[i] = travel_time +1
                """

            else: # cf.status == "waiting"
                cf.status = "moving"
                cf.drone.goTo(position_to_coords[cf.positions[0]], 0, travel_time)
                cf.positions.pop(0)
                next_moves[i] = travel_time +1
                

                # update times
                if len(cf.times) < 1:  
                    # if all times are used up, then mark drone as done
                    next_moves[i] = -1

    # minus 1 second from all next_moves (represents 1 second passing)
    next_moves = next_moves - np.full((1,len(all_drones)),1)[0]

    # increment timeslot
    t+=1