import math
import pandas as pd


class MapSetting:
    def __init__(self):
        # the array of cells, each is a dict to store its id and coordinates
        self.cells = None
        # the array of charging stations, each is a dict to store its id and coordinates
        self.stations = None
        # the edge length of a square map
        self.map_length = None
        # the total hovering time for each drone
        self.total_hover_time = None

        # the length of a cell
        self.cells_length = None
        # the number of cells in one edge of the map
        self.cells_num_edge = None
        # the radius of coverage range (to cover cells) of a charging station
        self.radius_coverage = None
        # the groups for coverage range of each charging station
        self.groups_station_coverage = None
        # the groups for neighbouring cells of each cell
        self.groups_neighbour_cells = None

    def setting_default(self, cells_num, stations_num, height, total_hover_time, map_length):
        """
        To set a map by using default setting, i.e., lined up the cells and charging stations given the number of cells
        and charging stations.
        :param cells_num: total number of cells
        :param stations_num: total number of charging stations
        :param height: hovering height of drones
        :param total_hover_time: the total hovering time for each drone
        :param map_length: the edge length of a square map
        :return:
        """
        self.map_length = map_length
        self.total_hover_time = total_hover_time
        # height = 659.2 / self.cells_num_edge

        self.cells_num_edge = int(math.sqrt(cells_num))  # number of cells at the edge of the map
        stations_num_edge = int(math.sqrt(stations_num))  # number of stations at the edge of the map
        self.cells_length = float(map_length / self.cells_num_edge)
        self.radius_coverage = self.cells_num_edge / stations_num_edge * self.cells_length * math.sqrt(2)

        # Set coordinates and id of all cells
        self.cells = []
        cell_id = 0
        for i in range(self.cells_num_edge):
            for j in range(self.cells_num_edge):
                x = self.cells_length / 2 + self.cells_length * i
                y = self.cells_length / 2 + self.cells_length * j
                z = height
                cell_dict = {'id': cell_id, 'x': x, 'y': y, 'z': z}
                self.cells.append(cell_dict)
                cell_id += 1

        # Set coordinates and id of all charging stations
        self.stations = []
        station_id = 0
        stations_length = float(map_length / (stations_num_edge - 1))
        for i in range(stations_num_edge):
            for j in range(stations_num_edge):
                x = stations_length * i
                y = stations_length * j
                z = 0
                station_dict = {'id': station_id, 'x': x, 'y': y, 'z': z}
                self.stations.append(station_dict)
                station_id += 1

        self.search_cells_in_coverage()

    def setting_read(self, path_map):
        """
        To set a map by reading the csv file that includes the map information.
        :param path_map: the path to the csv file.
        :return:
        """

        data_map = pd.read_csv(path_map)

        self.cells = []
        self.stations = []
        self.total_hover_time = 0
        min_x, max_x, min_y, max_y = 0, 0, 0, 0
        for i in range(data_map.shape[0]):
            item = data_map.iloc[i]
            item_id = int(item['id'])
            x = float(item['x'])
            y = float(item['y'])
            z = float(item['z'])
            value = float(item['value'])

            min_x = min(x, min_x)
            min_y = min(y, min_y)

            max_x = max(x, max_x)
            max_y = max(y, max_y)

            if item['type'] == 'SENSE':
                cell_dict = {'id': item_id, 'x': x, 'y': y, 'z': z, 'value': value}
                self.cells.append(cell_dict)
                self.total_hover_time += value

            elif item['type'] == 'BASE':
                station_dict = {'id': item_id, 'x': x, 'y': y, 'z': z}
                self.stations.append(station_dict)
            else:
                self.map_length = (x + y) / 2

        self.cells_num_edge = int(max_y - min_y)
        self.map_length = ((max_x - min_x) + (max_y - min_y))/2
        cells_num_edge = int(math.sqrt(len(self.cells)))
        stations_num_edge = int(math.sqrt(len(self.stations)))
        self.cells_length = float(self.map_length / cells_num_edge)
        self.radius_coverage = self.cells_num_edge / stations_num_edge * self.cells_length * math.sqrt(2)

        self.search_cells_in_coverage()

    def search_cells_in_coverage(self):
        """
        Find all neighbouring cells for each charging station, and store all of them into a dict.
        Find all neighbouring cells for each cell in the map, and store all of them into a dict.
        :return:
        """
        height = self.cells[0]['z']
        self.groups_neighbour_cells = {}
        self.groups_station_coverage = {}
        for station in self.stations:
            self.groups_station_coverage[station['id']] = []

        for cell in self.cells:
            # if the cell is within the coverage range of the charging station, then includes this cell
            for station in self.stations:
                distance = self.relative_distance_calc([station['x'], station['y'], height],
                                                       [cell['x'], cell['y'], cell['z']])
                if distance <= self.radius_coverage:
                    self.groups_station_coverage[station['id']].append(cell['id'])

            # find the neighbouring cells indexes of this cell
            cell_neighbours = []
            x_idx = cell['id'] % self.cells_num_edge
            y_idx = int(cell['id'] / self.cells_num_edge)
            if x_idx > 0:
                cell_neighbours.append(cell['id'] - 1)
            if x_idx < self.cells_num_edge:
                cell_neighbours.append(cell['id'] + 1)
            if y_idx > 0:
                cell_neighbours.append(cell['id'] - self.cells_num_edge)
            if y_idx < self.cells_num_edge:
                cell_neighbours.append(cell['id'] + self.cells_num_edge)
            if x_idx > 0 and y_idx > 0:
                cell_neighbours.append(cell['id'] - 1 - self.cells_num_edge)
            if x_idx < self.cells_num_edge and y_idx > 0:
                cell_neighbours.append(cell['id'] + 1 - self.cells_num_edge)
            if x_idx > 0 and y_idx < self.cells_num_edge:
                cell_neighbours.append(cell['id'] - 1 + self.cells_num_edge)
            if x_idx < self.cells_num_edge and y_idx < self.cells_num_edge:
                cell_neighbours.append(cell['id'] + 1 + self.cells_num_edge)
            self.groups_neighbour_cells[cell['id']] = cell_neighbours

    def setting_with_clustering(self, path_map, height):
        data_map = pd.read_csv(path_map)

        # define the clusters, each cluster has a station
        self.stations = []
        stations_coordinates = data_map.groupby('Label')[['Centroid_X', 'Centroid_Y']].mean()

        num_of_stations = data_map['Label'].max() + 1
        self.groups_station_coverage = {}
        for s in range(num_of_stations):
            self.groups_station_coverage[s] = []

            station_coord = stations_coordinates.loc[s]
            station_dict = {'id': s, 'x': station_coord['Centroid_X'], 'y': station_coord['Centroid_Y'], 'z': 0}
            self.stations.append(station_dict)

        self.cells = []
        self.total_hover_time = 0
        for i in range(data_map.shape[0]):
            item = data_map.iloc[i]
            item_id = int(item['id'])
            x = item['x']
            y = item['y']
            z = height
            cell_dict = {'id': item_id, 'x': x, 'y': y, 'z': z}
            self.cells.append(cell_dict)
            self.groups_station_coverage[item['Label']].append(item_id)

    def relative_distance_calc(self, point1, point2):
        """
        To calculate the relative distance between two points concerning to the coordinates of x, y and z.
        :param point1: coordinates of the first point
        :param point2: coordinates of the second point
        :return: relative distance
        """
        x1, y1, z1 = point1[0], point1[1], point1[2]
        x2, y2, z2 = point2[0], point2[1], point2[2]
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
