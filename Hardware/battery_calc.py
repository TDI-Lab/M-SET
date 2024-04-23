import os
original_cwd = os.getcwd()
import math
try:
    from Hardware_constants import *
    from cdca_epos_executor import get_coords, read_cdca_output, read_default_output
except:
    from Hardware.Hardware_constants import *
    from Hardware.cdca_epos_executor import get_coords, read_cdca_output, read_default_output
    

def calc_travel_time(start,end,speed):
    x_dist = (get_coords(end, USE_CELL_COORDS)[0]) - (get_coords(start, USE_CELL_COORDS)[0])
    y_dist = (get_coords(end, USE_CELL_COORDS)[1]) - (get_coords(start, USE_CELL_COORDS)[1])

    dist = math.sqrt((x_dist**2 + y_dist**2))

    time = dist / speed

    print(x_dist, y_dist, dist, speed, time)

    return time

def parse_input(input_path, input_mode, speed):
    positions = []
    hover_times = []
    flight_times = []

    #parse the input
    for drone in input_path:
        previous_position = None
        for position in drone:
            if input_mode == "cdca":
                print(position[0], previous_position)
                positions.append(position[0])
                hover_times.append(float(position[1])) # waiting time
                if previous_position != None:
                    flight_times.append(calc_travel_time(previous_position,position[0],speed)) # travel time
            elif input_mode == "default":
                positions.append(position)
                hover_times.append(SENSING_TIME) # sensing time
                if previous_position != None:
                    flight_times.append(calc_travel_time(previous_position,position,speed)) # travel time
            previous_position = position

    return positions, hover_times, flight_times

def calc_total_power(path, input_mode,speed):
    positions, hover_times, flight_times = parse_input(path, input_mode, speed)
    hover_power = sum([float(ht) for ht in hover_times])
    flight_power = sum([float(ft) for ft in flight_times])
    return hover_power, flight_power, hover_power+flight_power

if __name__ == '__main__':
    os.chdir(original_cwd)
    path = [[[[0.0, 0.0], 1], [[1.0, 1.0], 4], [[1.0, 2.0], 4], [[2.0, 1.0], 8], [[0.0, 0.0], 1]], [[[4.0, 0.0], 2]], [[[4.0, 3.0], 1], [[3.0, 2.0], 9], [[2.0, 2.0], 4], [[3.0, 1.0], 4], [[4.0, 3.0], 1]], [[[0.0, 3.0], 1], [[1.0, 2.0], 1], [[2.0, 2.0], 0], [[2.0, 1.0], 0], [[1.0, 1.0], 1], [[0.0, 3.0], 1]]]
    path = read_default_output("Hardware/epospaths/Final Demo/Basic cdca/4(3)cfnice.txt")
    input_mode = "default"
    speed = 0.1
    hover_power = 1.128405418267026 # W
    flight_power = hover_power # 1.128288294545468 # W
    hover_power, flight_power, total = calc_total_power(path, input_mode, speed)
    print("")
    print("Total power is %s W" % total)