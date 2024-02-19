import numpy as np

input = [[[[1,2],3],[[4,5],6]],[[[7,8],9],[[10,11],12]]]

print(input)

class Drone:
    def __init__(self):
        self.positions = []
        self.times = [] 
        self.status = "waiting" # moving, sensing, waiting

# Parameters
all_drones = []
next_moves = []
travel_time = 3
sensing_time = 0

#parse the input
for drone in input:
    d = Drone()
    all_drones.append(d)
    for position in drone:
        d.positions.append(position[0])
        #               set-off time (from previous cell) + travel time + sensing time + wait time
        #d.times.append(d.times[max(0,len(d.times)-1)]    + travel_time + sensing_time + position[1])
        next_moves.append(position[1])
    print(d.positions)


max_time = 0 
for d in all_drones:
    max_time = max(max_time, d.times[max(0,len(d.times)-1)])

print(max_time)

# Cycle through the time slots
# If a drone moves at that time slot, move it
for t in range(0,max_time):
    while np.any(next_moves[:] != -1):
        
        # minus 1 second from all next_moves (represents 1 second passing)
        next_moves - np.full((1,len(all_drones)),1)
        
        for i in range(0,len(all_drones)):
            if next_moves[i] == 0: # if it's time for the drone to change status (i.e. it has finished its current task)

                # Could change all_drones[i] to a variable drone=all_drones[i]. Would only work if implemented as a pointer, as otherwise the changes wouldn't be relayed back to all_drones
                # Could implement a move counter in Drone class e.g. drone.nmoves to track how many cells it had visited, then do drone.positions[nmoves] and times[nmoves] instead of costly(?) pop() operations
                # Sensing and waiting could technically be combined into one state of length (sensing_time + wait time) since they both currently just involve staying in the same spot, but have split them here to allow extension for different functionality in each state (e.g. to perform sensing actions) 

                if all_drones[i].status == "moving":
                    all_drones[i].status = "sensing"
                    next_moves[i] = sensing_time

                if all_drones[i].status == "sensing":
                    all_drones[i].status = "waiting"
                    next_moves[i] = all_drones[i].times[0]
                    all_drones[i].times[0].pop(0)

                    # if time was 0 then need to move straight to moving in this iteration too
                    if next_moves[i] == 0:
                        # Literally just copy-pasted from below
                        all_drones[i].status = "moving"
                        goTo(all_drones[i].positions[0], travel_time)
                        all_drones[i].positions.pop(0)

                else: # all_drones[i].status == "waiting"
                    all_drones[i].status = "moving"
                    goTo(all_drones[i].positions[0], travel_time)
                    all_drones[i].positions.pop(0)

                # update times
                if len(all_drones[i].times > 0):
                    next_moves[i] = all_drones[i].times[0]
                    all_drones[i].times.pop(0)
                else:
                    # if all times are used up, then mark drone as done
                    next_moves[i] = -1