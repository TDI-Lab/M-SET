import configparser
import itertools
import random
from pathlib import Path

import networkx as nx
import numpy as np

from path_generation.PlanGeneration.PowerConsumption import PowerConsumption


class RouteGeneration:
    def __init__(self):
        parent_path = Path(__file__).parent.resolve()
        config = configparser.ConfigParser()
        config.read(f'{parent_path}/conf/generation.properties')

        # output results
        self.visited_cells = None
        self.energy_consumption = None
        self.hover_time_arr = None
        self.distance_total = None   # meter
        self.flight_time = None    # second

        # plan information
        self.agent_id = None
        self.plan_id = None
        self.total_plans = int(config.get("plan", "planNum"))
        self.path_taken = None

        # power information
        self.battery_capacity = None
        self.hover_power = None
        self.flight_power = None
        self.ground_speed = None
        self.energy_efficiency = None

        # map information
        self.map = None
        self.cells = None
        self.stations = None

    def find_route(self, station_idx, path_mode, map_setting, plan_id):
        """
        To find a route or trajectory of a drone, including the departure/destination, indexes and the order of visited
        cells, the hovering time over each visited cell, the traveling distance, and the total energy consumption on
        traveling these visited cell.
        :param plan_id: the plan index
        :param station_idx: index of the charging (base) station
        :param path_mode: normal or greedy
        :param map_setting: the information of a sensing map
        :return:
        """
        self.map = map_setting
        self.cells = map_setting.cells
        self.stations = map_setting.stations

        # 1. start and end at the same charging station (departure and destination)
        station = self.stations[station_idx]

        # 2. find the indexes of visited cell randomly within the searching range
        self.visited_cells = []
        # find the searching range of a drone, that is, the indexes of cells close to the start/end charging station
        search_range_cells = self.map.groups_station_coverage[station['id']].copy()
        # choose the number of visited cells in a random way
        visited_cells_num = len(search_range_cells) if path_mode == "normal" else 1
        self.visited_cells = search_range_cells.copy()
        # if len(search_range_cells) >= visited_cells_num:
        #     self.visited_cells = random.sample(search_range_cells, visited_cells_num)
        # else:
        #     print(f"The searching range is too small! We need {visited_cells_num} "
        #           f"but only have {len(search_range_cells)}")
        #     return

        # 3. find the shortest path via visited cells using Dijkstra's algorithm based on TSP problem
        tsp_solution, distance_total = self.greedy_tsp(station)
        shortest_route_indexes = [int(x) for x in tsp_solution[1: -1]]
        self.distance_total = distance_total

        #  Compute parameters for energy calculations
        self.power_in_constant_params()
        #  Calculate the flight energy consumption (when the drone is moving)
        flight_energy = self.flight_power * distance_total / self.ground_speed

        #  Compute battery usage for path
        energy_utilisation = 1 - (plan_id / ((1./self.energy_efficiency) * self.total_plans))
        max_usage = self.battery_capacity * energy_utilisation
        while max_usage - flight_energy <= 0:
            max_usage = self.battery_capacity * random.random()

        # calculate the hover energy consumption
        hover_energy = max_usage - flight_energy
        # calculate the total energy consumption
        self.energy_consumption = max_usage
        # calculate the total sensing value of the path
        actual_sensing_total = hover_energy / self.hover_power

        #  Compute total hovering time of generated path
        visited_cells_dict = {}
        for cell in self.visited_cells:
            cell_dict = list(filter(lambda x: x["id"] == cell, self.map.cells))[0]
            visited_cells_dict[cell] = cell_dict
        total_path_requirement = sum(list(cell[1]["value"] for cell in visited_cells_dict.items()))

        #  Scale sensing values based on power consumption
        self.hover_time_arr = np.zeros(len(self.cells))
        for cell_id, cell_dict in visited_cells_dict.items():
            s_un = int((cell_dict["value"]/total_path_requirement)*actual_sensing_total)
            self.hover_time_arr[cell_id] = s_un

        # find the first cell randomly within the range, and remove the cell from the range
        first_cell_idx = random.choice(search_range_cells)
        search_range_cells.remove(first_cell_idx)
        self.visited_cells.append(first_cell_idx)
        self.path_taken = tsp_solution

    def power_in_constant_params(self):
        """
        To calculate the power consumption of hovering and flying by using the default (constant) parameters of
        ground speed, air density and air speed.
        :return:
        """
        power_consumption = PowerConsumption()
        power_consumption.set_params()
        power_consumption.calc_power()
        self.hover_power = power_consumption.hover_power
        self.flight_power = power_consumption.flight_power
        self.ground_speed = power_consumption.ground_speed
        self.battery_capacity = power_consumption.battery_capacity
        self.energy_efficiency = power_consumption.power_efficiency

    def greedy_tsp(self, station):
        g = nx.Graph()
        station_id = station["id"]
        station_name = f"station{station_id}"
        points = {station_name: (station['x'], station['y'])}
        for v_cell_id in self.visited_cells:
            cell = self.cells[v_cell_id]
            points[str(v_cell_id)] = (cell['x'], cell['y'])

        for p1, p2 in itertools.combinations(points.keys(), 2):
            distance = ((points[p1][0] - points[p2][0]) ** 2 + (points[p1][1] - points[p2][1]) ** 2) ** 0.5
            g.add_edge(p1, p2, weight=distance)

        tsp_solution = nx.approximation.greedy_tsp(g, source=station_name)
        # Calculate the total distance of the TSP route
        total_distance = sum(g[point1][point2]['weight'] for point1, point2 in zip(tsp_solution[:-1], tsp_solution[1:]))

        return tsp_solution, total_distance
