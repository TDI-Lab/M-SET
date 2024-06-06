import math
import pandas as pd
import numpy as np
import pyproj

N_COLUMNS = 3476

class PneumaCell:
    def __init__(self, x1, y1, x2, y2, total_time, num_vehicles_per_minute=None, id=None):
        
        self.cell = (x1, y1, x2, y2)
        self.centroid = self.get_centroid()
        if num_vehicles_per_minute is None:
            self.num_vehicles_per_minute = [0] * total_time
        else:
            self.num_vehicles_per_minute = num_vehicles_per_minute
        self.id = id
        self.vehicle_IDs_each_minute = [[] for _ in range(total_time)] # ids of vehicles in cell each minute

    def is_in_cell(self, x, y):
        x1, y1, x2, y2 = self.cell
        return x1 <= x <= x2 and y1 <= y <= y2
    
    def add_vehicle(self, time, vehicle_id):
        if vehicle_id in self.vehicle_IDs_each_minute[time]: # Dont count the same vehicle twice
            return
        
        self.vehicle_IDs_each_minute[time].append(vehicle_id)
        self.num_vehicles_per_minute[time] += 1

    def get_centroid(self):
        x1, y1, x2, y2 = self.cell
        return (x1 + x2) / 2, (y1 + y2) / 2
    
    def get_area(self):
        x1, y1, x2, y2 = self.cell
        return (x2 - x1) * (y2 - y1)
    
    def get_distance_from_other_cell(self, other_cell):
        x1, y1 = self.centroid
        x3, y3 = other_cell.centroid
        return math.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2)

    def __str__(self):
        return f'{self.cell}: {self.num_vehicles_per_minute}'
        

class PnuemaExperimentSetup:
    def __init__(self, data_path):
        self.data_path = data_path
        self.read_data()


    def generate_column_names(self, start, end):
        base_names = ['lat', 'lon', 'speed', 'lon_acc', 'lat_acc', 'time']
        return [f'{name}_{i}' for i in range(start, end+1) for name in base_names]
    
    def read_data(self):
        # Read the first 4 columns
        first_cols = ['track_id', 'type', 'traveled_d', 'avg_speed']
        self.full_data = pd.read_csv(self.data_path, sep=';',usecols=range(4), names=first_cols,skipinitialspace=True)
        
        #deep copy of vehicle_data
        self.multiIndexed_data = self.full_data.copy()
        # Calculate the number of chunks of 6 columns
        n = (3500 - 4) // 6

        t_per_chunk = 100
        # Read and process each chunk
        for i in range(n//t_per_chunk):
            chunk_cols = self.generate_column_names(i+1, i+t_per_chunk)
            chunk_data = pd.read_csv(self.data_path,sep=';', header=0, usecols=range(4+i*6, 4+(i+t_per_chunk)*6),skipinitialspace=True, names=chunk_cols, low_memory=False, memory_map=True)
        
            # Concatenate the chunk to the main DataFrame
            self.full_data = pd.concat([self.full_data, chunk_data], axis=1)

        # Print all traveled_d values
        print(self.full_data['traveled_d'])
        print(self.full_data['lat_1'])

    # Get largest time_n in chunk
    def get_largest_chunk_timestamp(self, end):
        return self.vehicle_data[f'time_{end}'].max()

    # def convert_lat_lon_to_xy(self):
    #     proj_wgs84 = pyproj.Proj(init='epsg:4326')
    #     proj_utm = pyproj.Proj(proj='utm', zone=34, datum='WGS84', south=False)  # south=False because it's in the Northern Hemisphere
    #     for i in range(4, self.full_data.shape[1], 6):
    #         # Get the column index for latitude and longitude
    #         lat_index = i
    #         lon_index = i + 1

    #         # Convert lat/lon to UTM
    #         x_utm, y_utm = pyproj.transform(proj_wgs84, proj_utm, self.full_data.iloc[:, lat_index], self.full_data.iloc[:, lon_index])

    #         # Update the column values with UTM coordinates
    #         self.full_data.iloc[:, lat_index] = x_utm
    #         self.full_data.iloc[:, lon_index] = y_utm

    #     return self.full_data
    def convert_lat_lon_to_xy(self):
        # Choose a reference point (this could be the centroid of your data)
        ref_lat = self.full_data.iloc[:, 4].mean()
        ref_lon = self.full_data.iloc[:, 5].mean()

        # Create a local tangent plane projection centered at the reference point
        proj_local = pyproj.Proj(proj='tmerc', lat_0=ref_lat, lon_0=ref_lon)

        for i in range(4, self.full_data.shape[1], 6):
            lat_index = i
            lon_index = i + 1

            # Convert lat/lon to local x/y
            x_local, y_local = pyproj.transform(pyproj.Proj(init='epsg:4326'), proj_local, self.full_data.iloc[:, lon_index], self.full_data.iloc[:, lat_index])

            # Update the column values with local x/y coordinates
            self.full_data.iloc[:, lat_index] = x_local
            self.full_data.iloc[:, lon_index] = y_local

        return self.full_data

    # get corner most points from dataset
    def get_corner_points(self, visualise=False):
        # Get the corner points of the dataset
        min_x = math.inf
        max_x = -math.inf
        min_y = math.inf
        max_y = -math.inf
        # For every column after the 4th column
        for i in range(4, self.full_data.shape[1], 6):
            # Get the min and max x and y coordinates
            min_x = min(min_x, self.full_data.iloc[:, i].min())
            max_x = max(max_x, self.full_data.iloc[:, i].max())
            min_y = min(min_y, self.full_data.iloc[:, i+1].min())
            max_y = max(max_y, self.full_data.iloc[:, i+1].max())
    
      
    
        # Plot the corner points in UTM
        if visualise:
            import matplotlib.pyplot as plt
            plt.scatter([min_x, max_x], [min_y, max_y])
            plt.xlabel('Easting (m)')
            plt.ylabel('Northing (m)')
            plt.title('Corner Points in UTM')
            plt.show(block=True)
    
        print(f"Min x: {min_x}, Max x: {max_x}, Min y: {min_y}, Max y: {max_y}")
        return min_x, max_x, min_y, max_y
    def put_grid_at_origin(self, min_x, max_x, min_y, max_y):
        width = max_x - min_x
        height = max_y - min_y
    
        # New grid starts at (0, 0) and has the same dimensions
        new_min_x = 0
        new_max_x = new_min_x + width
        new_min_y = 0
        new_max_y = new_min_y + height
    
        return new_min_x, new_max_x, new_min_y, new_max_y 
    # Create a quadrilateral and plit it into 6 cells
    def create_cells_from_corners(self, min_x, max_x, min_y, max_y, at_origin=False):
        
        if at_origin:
            min_x, max_x, min_y, max_y = self.put_grid_at_origin(min_x, max_x, min_y, max_y)
       
       
        # Push out the corners by 5%
        x_margin = (max_x - min_x) * 0.01
        y_margin = (max_y - min_y) * 0.01
        min_x -= x_margin
        max_x += x_margin
        min_y -= y_margin
        max_y += y_margin

        # Create a grid of cells
        n = 2
        m = 3
        x_step = (max_x - min_x) / n
        y_step = (max_y - min_y) / m
    
        # Create a list of cells
        cells = []
        import matplotlib.pyplot as plt
        for i in range(n):
            for j in range(m):
                # Get the coordinates of the cell
                x1 = min_x + i * x_step
                x2 = min_x + (i + 1) * x_step
                y1 = min_y + j * y_step
                y2 = min_y + (j + 1) * y_step
    
                # Add the cell to the list
                cells.append((x1, y1, x2, y2))
                
                # Plot the cell
                plt.plot([x1, x2, x2, x1, x1], [y1, y1, y2, y2, y1], 'r-')
                plt.scatter([x1, x2, x2, x1], [y1, y1, y2, y2], color='black')
                plt.xlabel('Easting (m)')
                plt.ylabel('Northing (m)')
                plt.title('Cells in UTM')
                plt.show(block=False)
                plt.pause(1)  # Pause for 0.5 seconds before plotting the next cell
        return cells
    
    # plot all cells and positions on them
    def plot_cells_and_positions(self, cells):


        # Plot the cells
        import matplotlib.pyplot as plt
        for cell in cells:
            x1, y1, x2, y2 = cell.cell
            plt.plot([x1, x2, x2, x1, x1], [y1, y1, y2, y2, y1], 'r-')
        # plt.show(block=True)

        

        # Plot the positions
        for i in range(4, self.full_data.shape[1], 6):
            plt.scatter(self.full_data.iloc[:, i], self.full_data.iloc[:, i+1])

        plt.xlabel('Easting (m)')
        plt.ylabel('Northing (m)')
        plt.title('Cells and Positions in UTM')
        plt.show(block=True)

    # Find the cell that a point is in
    def find_cell(self, x, y, cells):
        for i, cell in enumerate(cells):
            if cell.is_in_cell(x, y):
                return i
        return None

    # generate distribution of how many drones are in each cell per minute
    def generate_distribution(self):
        total_time = self.get_max_time()
        total_mins = (total_time // 60) + 1

        # Convert the lat/lon positions to UTM
        self.convert_lat_lon_to_xy()

        min_x, max_x, min_y, max_y = self.get_corner_points()

        # Create a grid of cells
        cells = self.create_cells_from_corners(min_x, max_x, min_y, max_y)

        # Create a list of PneumaCell objects
        pneuma_cells = [PneumaCell(*cell, int(total_mins)) for cell in cells]

        # For each time point, find the cell that each vehicle is in and increment the count
        for i in range(4, self.full_data.shape[1], 6):
            for j in range(self.full_data.shape[0]):
                x = self.full_data.iloc[j, i]
                y = self.full_data.iloc[j, i+1]
                vehicle_id = self.full_data.iloc[j, 0]
                cell_index = self.find_cell(x, y, pneuma_cells)
                if cell_index is not None:
                    time = self.full_data.iloc[j, i+5]

                    if np.isnan(time):
                        continue
                    minute = int(time //  60)
                    pneuma_cells[cell_index].add_vehicle(minute, vehicle_id)

        self.plot_cells_and_positions(pneuma_cells)
        # Write data to a file
        with open('output.txt', 'w') as f:
            # header
            f.write('Max time: ' + str(total_time) + '\n')
            f.write('Cell: [x1, y1, x2, y2]: [vehicles per minute]\n')
            for cell in pneuma_cells:
                print(cell.get_area())
                f.write(str(cell) + '\n')
        
        return pneuma_cells

    def get_max_time(self):
        max_time = 0

        for i in range(4, self.full_data.shape[1], 6):
            max_time = max(max_time, self.full_data.iloc[:, i+5].max())

        return max_time
        
        
         


if __name__ == '__main__':

    data_path = "C:/Users/Alex/Downloads/20181024_d9_0830_0900 (1).csv"
    pnuema_experiments = PnuemaExperimentSetup(data_path)

    dist = pnuema_experiments.generate_distribution()

    for cell in dist:
        print(cell)
    

