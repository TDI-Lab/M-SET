from os import listdir
from pprint import pprint

from matplotlib import pyplot as plt

vehicle_entries = []


def load_entries():
    files = listdir("/home/c41/Downloads/Plans_for_Drones/Plans_for_Drones/RealData_Store/20181024_0830_0900")
    files = list(filter(lambda x: x[-4:] == ".csv", files))
    files.sort()
    for file in files:
        with open(f"/home/c41/Downloads/Plans_for_Drones/"
                  f"Plans_for_Drones/RealData_Store/20181024_0830_0900/{file}") as v_file:
            lines = v_file.readlines()
        lines = lines[1:]
        for line in lines:
            entry = list(map(float, line.strip("\n").split(",")[1:]))
            vehicle_entries.append(entry)


#  generate the maximum and minimum sensing points on the map
def min_max_locs():
    global vehicle_entries
    potential_bounds = []
    # files = listdir("/home/c41/Downloads/Plans_for_Drones/Plans_for_Drones/RealData_Store/20181024_0830_0900")
    # files = list(filter(lambda x: x[-4:] == ".csv", files))
    # files.sort()
    # for file in files:
    #     with open(f"/home/c41/Downloads/Plans_for_Drones/"
    #               f"Plans_for_Drones/RealData_Store/20181024_0830_0900/{file}") as v_file:
    #         lines = v_file.readlines()
    #     lines = lines[1:]
    #     for line in lines:
    #         entry = list(map(float, line.strip("\n").split(",")[1:]))
    #         vehicle_entries.append(entry)
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


def create_sensing_grid(min, max):
    long_step = 30
    lat_step = 30
    grid = [[] for _ in range(0, lat_step)]
    for i in range(0, lat_step):
        for j in range(0, long_step):
            long_mod = j / long_step
            lat_mod = i / lat_step
            coord = (min[0] * (1 - long_mod) + max[0] * long_mod, min[1] * (1 - lat_mod) + max[1] * lat_mod)
            grid[i].append(coord)
    return grid


def convert_grid_to_meters(grid):
    converted_grid = [[] for _ in grid]
    base = grid[0][0]
    for i, row in enumerate(grid):
        for cell in row:
            cell_in_meters = ((cell[0] - base[0]) * 111111, (cell[1] - base[1]) * 111111)
            converted_grid[i].append(cell_in_meters)
    return converted_grid, grid[0][0]


#  id,timeBegin,timeEnd,lonBegin,latBegin,lonEnd,latEnd,area
def discrete_vehicle_movements(pos_grid, long_lat_base):
    global vehicle_entries
    distribution_grid = [[0 for _ in pos_grid[0]] for _ in pos_grid]
    for entry in vehicle_entries:
        start_time = round(entry[1])
        end_time = round(entry[2])
        start_pos = (entry[3], entry[4])
        end_pos = (entry[5], entry[6])
        total_moves = end_time - start_time
        positions = []
        for i in range(0, total_moves):
            step = i / total_moves
            long_pos = start_pos[0] * (1 - step) + end_pos[0] * step
            lat_pos = start_pos[1] * (1 - step) + start_pos[1] * step
            positions.append((long_pos, lat_pos))
        for pos in positions:
            #  calculate i
            i_pos = 0
            for i in range(1, len(pos_grid[0])):
                in_meters = (pos[0]-long_lat_base[0])*111111
                if pos_grid[0][i][0] > in_meters:
                    i_pos = i
                    break
            #  calculate j
            j_pos = 0
            for j in range(1, len(pos_grid)):
                in_meters = (pos[1]-long_lat_base[1])*111111
                if pos_grid[j][0][1] > in_meters:
                    j_pos = j
                    break
            distribution_grid[j_pos][i_pos] += 1
    return distribution_grid


def write_sensing_to_file(sense_grid, loc_grid):
    lines = ["type,id,x,y,z,value\n"]
    cell_id = 0
    for j, row in enumerate(loc_grid):
        for i, cell in enumerate(row):
            if sense_grid[j][i] == 0:
                continue
            new_row = f"SENSE,{cell_id},{loc_grid[i][j][0]},{loc_grid[i][j][1]},1.0,{sense_grid[j][i]}\n"
            lines.append(new_row)
            cell_id += 1
    #  arbitrary corner base stations
    base_offset = 30.
    base_stations = [
        f"BASE,0,{loc_grid[0][0][0] - base_offset},{loc_grid[0][0][1] - base_offset},0,0\n",
        f"BASE,1,{loc_grid[0][len(loc_grid[0])-1][0] + base_offset},{loc_grid[0][0][1] - base_offset},0,0\n",
        f"BASE,2,{loc_grid[len(loc_grid)-1][0][0] - base_offset},{loc_grid[len(loc_grid)-1][0][1] + base_offset},0,0\n",
        f"BASE,3,{loc_grid[len(loc_grid)-1][len(loc_grid[0])-1][0] + base_offset},"
        f"{loc_grid[len(loc_grid)-1][len(loc_grid[0])-1][1] + base_offset},0,0\n",
    ]
    for station in base_stations:
        lines.append(station)
    with open("/home/c41/Drones-Testbed/examples/pneuma_grid.csv", "w") as file:
        file.writelines(lines)


if __name__ == '__main__':
    load_entries()
    grid, orig_base = convert_grid_to_meters(create_sensing_grid(*min_max_locs()))
    vehicle_movements = discrete_vehicle_movements(grid, orig_base)
    write_sensing_to_file(vehicle_movements, grid)
