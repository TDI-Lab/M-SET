import math
import os
original_cwd = os.getcwd()
try:
    from Hardware_constants import *
except:
    from Hardware.Hardware_constants import * 
from cdca_epos_executor import get_coords, read_cdca_output, read_default_output

def calc_travel_time(start,end,speed):
    if start != None:
        x_dist = (get_coords(end, USE_CELL_COORDS)[0]) - (get_coords(start, USE_CELL_COORDS)[0])
        y_dist = (get_coords(end, USE_CELL_COORDS)[1]) - (get_coords(start, USE_CELL_COORDS)[1])

        dist = math.sqrt((x_dist**2 + y_dist**2))

        time = dist / speed

        #print(x_dist, y_dist, dist, speed, time)

        return time
    else:
        return 0

def parse_input(input_path, input_mode, speed):
    times = []
    positions = []

    #parse the input
    for drone in input_path:
        previous_position=None
        for position in drone:
            if input_mode == "cdca":
                positions.append(position[0])
                times.append(int(position[1])) # waiting time
                times.append(calc_travel_time(previous_position,position,speed)) # travel time
            elif input_mode == "default":
                positions.append(position)
                times.append(SENSING_TIME) # sensing time
                times.append(calc_travel_time(previous_position,position,speed)) # travel time
            previous_position = position

    return positions, times

os.chdir(original_cwd)
path = read_default_output("Hardware/epospaths/epos3_demo.txt")
parsed_times = parse_input(path, "default", 0.1)[1]
print(parsed_times)
print(sum(parsed_times))