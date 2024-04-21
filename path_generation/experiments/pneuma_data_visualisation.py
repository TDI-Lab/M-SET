from os import listdir
from pprint import pprint
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


#  /home/c41/Downloads/Plans_for_Drones/Plans_for_Drones/RealData_Store


def main():
    with open("/home/c41/Downloads/Plans_for_Drones/"
              "Plans_for_Drones/RealData_Store/20181024_0830_0900/Car.csv") as file:
        lines = file.readlines()
    lines = lines[1:]
    car_entries = []
    for line in lines:
        entry = list(map(float, line.strip("\n").split(",")[1:]))
        car_entries.append(entry)
    car_entries = list(filter(lambda x: x[7] == 1.0, car_entries))
    car_entries.sort(key=lambda x: x[1])
    latest_end_time = round(max(car_entries, key=lambda x: x[2])[2])
    points_per_frame = [[] for _ in range(0, latest_end_time)]
    for entry in car_entries:
        start_time = round(entry[1])
        end_time = round(entry[2])
        # print(f"Entry ID: {entry[0]}, Start Time: {start_time}, End Time: {end_time}")
        start_pos = (entry[3], entry[4])
        end_pos = (entry[5], entry[6])
        positions = []
        num_steps = end_time - start_time
        for i in range(0, num_steps):
            step = i / num_steps
            step_pos_long = start_pos[0] * (1 - step) + end_pos[0] * step
            step_pos_lat = start_pos[1] * (1 - step) + end_pos[1] * step
            positions.append((step_pos_long, step_pos_lat))
        for frame_id, point in zip(range(start_time, end_time), positions):
            points_per_frame[frame_id].append(point)

    pprint(points_per_frame)


def min_max_locs():
    potential_bounds = []
    with open("/home/c41/Downloads/Plans_for_Drones/"
              "Plans_for_Drones/RealData_Store/20181024_0830_0900/Car.csv") as file:
        lines = file.readlines()
    lines = lines[1:]
    vehicle_entries = []
    for line in lines:
        entry = list(map(float, line.strip("\n").split(",")[1:]))
        vehicle_entries.append(entry)
    for i in range(1, 11):
        #  filter for area
        entries = list(filter(lambda x: x[7] == float(i), vehicle_entries))
        #  find minimum start long and lat
        min_long_start = min(entries, key=lambda x: x[3])[3]
        min_lat_start = min(entries, key=lambda x: x[4])[4]
        #  find maximum start long and lat
        max_long_start = max(entries, key=lambda x: x[3])[3]
        max_lat_start = max(entries, key=lambda x: x[4])[4]
        #  find minimum end long and lat
        min_long_end = min(entries, key=lambda x: x[5])[5]
        min_lat_end = min(entries, key=lambda x: x[6])[6]
        #  find maximum end long and lat
        max_long_end = max(entries, key=lambda x: x[5])[5]
        max_lat_end = max(entries, key=lambda x: x[6])[6]

        #  bundle
        min_start_pos = (min_long_start, min_lat_start)
        max_start_pos = (max_long_start, max_lat_start)
        min_end_pos = (min_long_end, min_lat_end)
        max_end_pos = (max_long_end, max_lat_end)
        potential_bounds.append(min_start_pos)
        potential_bounds.append(max_start_pos)
        potential_bounds.append(min_end_pos)
        potential_bounds.append(max_end_pos)

        #  plot min and max values for each
        plt.scatter(min_start_pos[0], min_start_pos[1], c="b")
        plt.scatter(max_start_pos[0], max_start_pos[1], c="r")
        plt.scatter(min_end_pos[0], min_end_pos[1], c="g")
        plt.scatter(max_end_pos[0], max_end_pos[1], c="y")
        plt.ticklabel_format(useOffset=False)

    #  compute the bounds of the grid
    min_long = min(potential_bounds, key=lambda x: x[0])[0]
    max_long = max(potential_bounds, key=lambda x: x[0])[0]
    min_lat = min(potential_bounds, key=lambda x: x[1])[1]
    max_lat = max(potential_bounds, key=lambda x: x[1])[1]
    return (min_long, min_lat), (max_long, max_lat)


def create_zone_sensing_grid():
    #  zone 1 bounds (long, lat)
    axis_min_real = (23.734619, 37.976679)
    axis_max_real = (23.738652, 37.981113)
    #  convert to meters
    #  zone 1 bounds, origin is cell 0, the bottom left corner of the area
    axis_max = ((axis_max_real[0]-axis_min_real[0])*111111, (axis_max_real[1]-axis_min_real[1])*111111)
    axis_min = (0, 0)
    steps_width = 15
    steps_height = 15
    grid = [[] for i in range(0, steps_height)]
    for i in range(0, steps_height):
        height_step = i/steps_height
        for j in range(0, steps_width):
            width_step = j/steps_width
            pos_long = axis_min[0]*(1-width_step) + width_step*axis_max[0]
            pos_lat = axis_min[1]*(1-height_step) + height_step*axis_max[1]
            grid[i].append((pos_long, pos_lat))
            plt.ticklabel_format(useOffset=False)
            plt.scatter(pos_long, pos_lat, c="b", marker="s")
    base_station_offset = 30.
    base0 = (grid[0][0][0]-base_station_offset, grid[0][0][1]-base_station_offset)
    base1 = (grid[0][steps_width-1][0]+base_station_offset, grid[0][steps_width-1][1]-base_station_offset)
    base2 = (grid[0][0][0]-base_station_offset, grid[steps_height-1][0][1]+base_station_offset)
    base3 = (grid[0][steps_width-1][0]+base_station_offset, grid[steps_height-1][0][1]+base_station_offset)
    plt.scatter(base0[0], base0[1], c="b", marker="c")
    plt.scatter(base1[0], base1[1], c="b", marker="c")
    plt.scatter(base2[0], base2[1], c="b", marker="c")
    plt.scatter(base3[0], base3[1], c="b", marker="c")
    #  write grid to file
    examples_dir = Path(__file__).parent.resolve()
    examples_dir = f"{examples_dir}/../../examples"
    write_loc = f"{examples_dir}/pneuma_zone_1.csv"
    #  create rows
    rows = ["type,id,x,y,z,value\n"]
    count = 0
    for row in grid:
        for cell in row:
            new_row = f"SENSE,{count},{cell[0]},{cell[1]},1.0,30.0\n"
            rows.append(new_row)
            count += 1
    rows.append(f"BASE,0,{base0[0]},{base0[1]},0,0\n")
    rows.append(f"BASE,1,{base1[0]},{base1[1]},0,0\n")
    rows.append(f"BASE,2,{base2[0]},{base2[1]},0,0\n")
    rows.append(f"BASE,3,{base3[0]},{base3[1]},0,0\n")
    with open(write_loc, "w") as file:
        for row in rows:
            file.writelines(row)
    plt.show()


if __name__ == '__main__':
    print(min_max_locs())
    plt.show()
