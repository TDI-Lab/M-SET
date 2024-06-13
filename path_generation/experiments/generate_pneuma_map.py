from os import listdir
from pprint import pprint
from typing import Union, List

from matplotlib import pyplot as plt

vehicle_entries = []


output_file_name = ""

MAXIMUM_SENSING = 1000


def load_entries(vehicles: Union[str, List[str]] = "all"):
    global vehicle_entries
    vehicle_entries = []
    files = listdir("/home/c41/Downloads/Plans_for_Drones/Plans_for_Drones/RealData_Store/20181024_0830_0900")
    files = list(filter(lambda x: x[-4:] == ".csv", files))
    if vehicles != "all":
        filtered_files = []
        for vehicle in vehicles:
            filtered_files += list(filter(lambda x: x.lower()[0:len(vehicle)] == vehicle.lower(), files))
        files = filtered_files
    files.sort()
    for file in files:
        with open(f"/home/c41/Downloads/Plans_for_Drones/"
                  f"Plans_for_Drones/RealData_Store/20181024_0830_0900/{file}") as v_file:
            lines = v_file.readlines()
        lines = lines[1:]
        for line in lines:
            entry = list(map(float, line.strip("\n").split(",")[1:]))
            vehicle_entries.append(entry)
    print(len(vehicle_entries))


#  generate the maximum and minimum sensing points on the map
def min_max_locs():
    global vehicle_entries
    potential_bounds = []
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


def min_max_locs_by_area():
    global vehicle_entries
    area_bounds = {}
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

        area_bounds[i] = {"min_start": min_start_pos, "max_start": max_start_pos,
                          "min_end": min_end_pos, "max_end": max_end_pos}

        #  plot min and max values for each
        plt.scatter(min_start_pos[0], min_start_pos[1], c="b")
        plt.scatter(max_start_pos[0], max_start_pos[1], c="r")
        plt.scatter(min_end_pos[0], min_end_pos[1], c="g")
        plt.scatter(max_end_pos[0], max_end_pos[1], c="y")
        plt.ticklabel_format(useOffset=False)

    #  compute the bounds of the grid
    for area, points in area_bounds.items():
        min_long = min(points["min_start"][0], points["min_end"][0])
        max_long = min(points["max_start"][0], points["max_end"][0])
        min_lat = min(points["min_start"][1], points["min_end"][1])
        max_lat = min(points["max_start"][1], points["max_end"][1])
        area_bounds[area] = {"min": (min_long, min_lat), "max": (max_long, max_lat)}
    return area_bounds


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


def discrete_movement(t_start, t_end, p_start, p_end):
    total_moves = t_end - t_start
    positions = []
    for i in range(0, total_moves):
        step = i / total_moves
        long_pos = p_start[0] * (1 - step) + p_end[0] * step
        lat_pos = p_start[1] * (1 - step) + p_end[1] * step
        positions.append((long_pos, lat_pos))
    return positions


#  id,timeBegin,timeEnd,lonBegin,latBegin,lonEnd,latEnd,area
def discrete_vehicle_movements(pos_grid, long_lat_base):
    global vehicle_entries
    num_points = 0
    distribution_grid = [[0 for _ in pos_grid[0]] for _ in pos_grid]
    for entry in vehicle_entries:
        start_time = round(entry[1])
        end_time = round(entry[2])
        start_pos = (entry[3], entry[4])
        end_pos = (entry[5], entry[6])
        positions = discrete_movement(start_time, end_time, start_pos, end_pos)
        for pos in positions:
            #  calculate i
            i_pos = 0
            for i in range(1, len(pos_grid[0])):
                in_meters = (pos[0] - long_lat_base[0]) * 111111
                if pos_grid[0][i][0] > in_meters:
                    i_pos = i
                    break
            #  calculate j
            j_pos = 0
            for j in range(1, len(pos_grid)):
                in_meters = (pos[1] - long_lat_base[1]) * 111111
                if pos_grid[j][0][1] > in_meters:
                    j_pos = j
                    break
            distribution_grid[j_pos][i_pos] += 1
            num_points += 1
    #  normalise distribution grid
    for i, row in enumerate(distribution_grid):
        for j, cell in enumerate(row):
            normal = (float(cell)/num_points)*MAXIMUM_SENSING
            distribution_grid[i][j] = round(normal, 3)
    return distribution_grid


def sensing_by_area():
    total_sensing = {}
    for entry in vehicle_entries:
        if int(entry[7]) not in total_sensing:
            total_sensing[int(entry[7])] = 1
        else:
            total_sensing[int(entry[7])] += 1
    #  normalise points
    for area, sensing in total_sensing.items():
        total_sensing[area] = (float(sensing)/len(vehicle_entries))*MAXIMUM_SENSING
    return total_sensing


def write_sensing_grid_to_file(sense_grid, loc_grid):
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
        f"BASE,1,{loc_grid[0][len(loc_grid[0]) - 1][0] + base_offset},{loc_grid[0][0][1] - base_offset},0,0\n",
        f"BASE,2,{loc_grid[len(loc_grid) - 1][0][0] - base_offset},{loc_grid[len(loc_grid) - 1][0][1] + base_offset},0,0\n",
        f"BASE,3,{loc_grid[len(loc_grid) - 1][len(loc_grid[0]) - 1][0] + base_offset},"
        f"{loc_grid[len(loc_grid) - 1][len(loc_grid[0]) - 1][1] + base_offset},0,0\n",
    ]
    for station in base_stations:
        lines.append(station)
    with open(f"/home/c41/Drones-Testbed/examples/pneuma_grid{output_file_name}.csv", "w") as file:
        file.writelines(lines)


def write_sensing_points_to_file(points, sensing, origin, loc_grid):
    lines = ["type,id,x,y,z,value\n"]
    cell_id = 0
    for cell, position in points.items():
        long = (position[0] - origin[0]) * 111111
        lat = (position[1] - origin[1]) * 111111
        new_row = f"SENSE,{cell_id},{long},{lat},1,{sensing[cell]}\n"
        lines.append(new_row)
        cell_id += 1
    #  arbitrary corner base stations
    base_offset = 30.
    base_stations = [
        f"BASE,0,{loc_grid[0][0][0] - base_offset},{loc_grid[0][0][1] - base_offset},0,0\n",
        f"BASE,1,{loc_grid[0][len(loc_grid[0]) - 1][0] + base_offset},{loc_grid[0][0][1] - base_offset},0,0\n",
        f"BASE,2,{loc_grid[len(loc_grid) - 1][0][0] - base_offset},{loc_grid[len(loc_grid) - 1][0][1] + base_offset},0,0\n",
        f"BASE,3,{loc_grid[len(loc_grid) - 1][len(loc_grid[0]) - 1][0] + base_offset},"
        f"{loc_grid[len(loc_grid) - 1][len(loc_grid[0]) - 1][1] + base_offset},0,0\n",
    ]
    for station in base_stations:
        lines.append(station)
    with open(f"/home/c41/Drones-Testbed/examples/pneuma_points{output_file_name}.csv", "w") as file:
        file.writelines(lines)


def create_pneuma_grid_map(vehicles: Union[str, List[str]] = "all"):
    load_entries(vehicles=vehicles)
    grid, orig_base = convert_grid_to_meters(create_sensing_grid(*min_max_locs()))
    vehicle_movements = discrete_vehicle_movements(grid, orig_base)
    write_sensing_grid_to_file(vehicle_movements, grid)


def create_pneuma_point_map(vehicles: Union[str, List[str]] = "all"):
    load_entries(vehicles=vehicles)
    grid, orig_base = convert_grid_to_meters(create_sensing_grid(*min_max_locs()))
    area_areas = min_max_locs_by_area()
    area_centers = {}
    for area, points in area_areas.items():
        area_centers[area] = ((points["min"][0] + points["max"][0]) / 2., (points["min"][1] + points["max"][1]) / 2.)
    area_sensing = sensing_by_area()
    write_sensing_points_to_file(area_centers, area_sensing, orig_base, grid)


if __name__ == '__main__':
    from visualise_maps import visualise_map_by_path, visualise_sensing
    create_pneuma_grid_map(vehicles=["Bus", "Car", "HeavyVehicle", "MediumVehicle", "Motorcycle", "Taxi"])
    create_pneuma_point_map(vehicles=["Bus", "Car", "HeavyVehicle", "MediumVehicle", "Motorcycle", "Taxi"])
    # vehicle_types = ["Bus", "Car", "HeavyVehicle", "MediumVehicle", "Motorcycle", "Taxi"]
    # for vehicle in vehicle_types:
    #     output_file_name = f"_{vehicle}"
    #     create_pneuma_point_map(vehicles=[vehicle])


