import math
import os
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
    
    # def read_data(self):
    #     # df = pd.read_csv(self.data_path, sep=';', nrows=5)
    #     # total_cols = len(df.columns)
    #     dtypes = {
    #         'track_id': 'int64',  # replace with the actual data type
    #         'type': 'str',  # replace with the actual data type
    #         'traveled_d': 'float64',  # replace with the actual data type
    #         'avg_speed': 'float64'  # replace with the actual data type
    #     }

    #     # Read the first 4 columns
    #     first_cols = ['track_id', 'type', 'traveled_d', 'avg_speed']
    #     self.full_data = pd.read_csv(self.data_path, sep=';',usecols=range(4), names=first_cols,skipinitialspace=True)
        
    #     #deep copy of vehicle_data
    #     self.multiIndexed_data = self.full_data.copy()
    #     # Calculate the number of chunks of 6 columns
    #     n = (50000 - 4) // 6

    #     t_per_chunk = 120 * 5 #Multiple of 6
    #     # Read and process each chunk
    #     for i in range(n//t_per_chunk):
    #         chunk_cols = self.generate_column_names(i+1, i+t_per_chunk)
    #         chunk_data = pd.read_csv(self.data_path,sep=';', usecols=range(4+i*6, 4+(i+t_per_chunk)*6),skipinitialspace=True, skiprows=1 ,names=chunk_cols, low_memory=True, memory_map=True, dtype=float)
    #         # chunk_data = pd.read_csv(self.data_path, sep=';', usecols=range(4+i*6, 4+(i+t_per_chunk)*6), skiprows=1, skipinitialspace=True, low_memory=False, memory_map=True, dtype=float)

    #         # Assign the headers to the data
    #         chunk_data.columns = chunk_cols
    #         # Reset the index of chunk_data
    #         chunk_data.reset_index(drop=True, inplace=True)

    #         # Concatenate the chunk to the main DataFrame
    #         self.full_data = pd.concat([self.full_data, chunk_data], axis=1)            
    #         # n =0
    #         #if all final columns are nan
    #         # while True:
    #         #     if self.full_data.iloc[:, -1].isnull().all():
    #         #         self.full_data = self.full_data.iloc[:, :-1]
    #         #         print(f'dropped {n} columns')
    #         #         n += 1
    #         #     else:
    #         #         break
    #     # self.full_data = self.remove_trailing_nan(self.full_data)
    #     # print the last value of the 1st row
    #     print(self.full_data.iloc[46, -1])
    #     # Print all traveled_d values
    #     print(self.full_data['traveled_d'])
    #     print(self.full_data['lat_1'])
    def read_data(self):
        delimiter = ';'
    
        with open(self.data_path, 'r') as file:
            lines = file.readlines()
            vehicle_info = []
            trajectory_info = []
            # Jumping the first line which is the header
            lines = lines[1:]
            for line in lines:
                # Removing the \n at the end of the line
                line = line.strip('\n').strip(' ')
                contents = line.split(delimiter)
                # Removing the white spaces
                contents = [contents[i].strip() for i in range(len(contents))]
    
                vehicle_info.append(contents[:4])
    
                k = 4  # Skipping the first 4 columns which are track_id, type, traveled_d, avg_speed
                for i in range(k, len(contents), 6):
                    # Concatenating the track_id with the trajectory information
                    trajectory_info.append([contents[0], *contents[i:i+6]])
    
        self.vehicle_df = pd.DataFrame(data=vehicle_info, columns=['track_id', 'type', 'traveled_d', 'avg_speed'])
        self.traj_df = pd.DataFrame(data=trajectory_info, columns=['track_id', 'lat', 'lon', 'speed', 'lon_acc', 'lat_acc', 'time'])
        self.traj_df.dropna(subset=['lat', 'lon', 'speed', 'lon_acc', 'time'], inplace=True)

        # Convert 'track_id' to int
        self.traj_df['track_id'] = self.traj_df['track_id'].astype(int)

        # Convert the rest of the columns to float
        for col in ['lat', 'lon', 'speed', 'lon_acc', 'lat_acc', 'time']:
            self.traj_df[col] = self.traj_df[col].astype(float)
        # Dropping any rows with NaN values
    
        # Concatenating the two dataframes

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
        ref_lat = self.traj_df.iloc[:, 1].mean()
        ref_lon = self.traj_df.iloc[:, 2].mean()

        # Create a local tangent plane projection centered at the reference point
        proj_local = pyproj.Proj(proj='tmerc', lat_0=ref_lat, lon_0=ref_lon)

        # for i in range(4, self.full_data.shape[1], 6):
        lat_index = 1
        lon_index = 2

        # Convert lat/lon to local x/y
        x_local, y_local = pyproj.transform(pyproj.Proj(init='epsg:4326'), proj_local, self.traj_df.iloc[:, lon_index], self.traj_df.iloc[:, lat_index])

        # Update the column values with local x/y coordinates
        self.traj_df.iloc[:, lat_index] = x_local
        self.traj_df.iloc[:, lon_index] = y_local

        return self.traj_df

    # get corner most points from dataset
    def get_corner_points(self, visualise=False):
        # Get the corner points of the dataset
        min_x = math.inf
        max_x = -math.inf
        min_y = math.inf
        max_y = -math.inf
        # For every column after the 4th column
        # for i in range(4, self.full_data.shape[1], 6):
            # Get the min and max x and y coordinates
        min_x = min(min_x, self.traj_df.iloc[:, 1].min())
        max_x = max(max_x, self.traj_df.iloc[:, 1].max())
        min_y = min(min_y, self.traj_df.iloc[:, 2].min())
        max_y = max(max_y, self.traj_df.iloc[:, 2].max())
    
      
    
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
                # plt.plot([x1, x2, x2, x1, x1], [y1, y1, y2, y2, y1], 'r-')
                # plt.scatter([x1, x2, x2, x1], [y1, y1, y2, y2], color='black')
                # plt.xlabel('Easting (m)')
                # plt.ylabel('Northing (m)')
                # plt.title('Cells in UTM')
                # plt.show(block=False)
                # plt.pause(1)  # Pause for 0.5 seconds before plotting the next cell
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
        # for i in range(4, self.full_data.shape[1], 6):
        plt.scatter(self.traj_df.iloc[:, 1], self.traj_df.iloc[:, 2])

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
    # def generate_distribution(self, output_name='output.txt'):
    #     total_time = float(self.get_max_time())
    #     total_mins = (total_time // 60) + 1

    #     # Convert the lat/lon positions to UTM
    #     self.convert_lat_lon_to_xy()

    #     min_x, max_x, min_y, max_y = self.get_corner_points()

    #     # Create a grid of cells
    #     cells = self.create_cells_from_corners(min_x, max_x, min_y, max_y)
    #     print("starting vehicle count")
    #     # Create a list of PneumaCell objects
    #     pneuma_cells = [PneumaCell(*cell, int(total_mins)) for cell in cells]
    #     # For each time point, find the cell that each vehicle is in and increment the count
    #     # for i in range(4, self.full_data.shape[1], 6):
    #     for j in range(1, self.traj_df.shape[0]):
    #         x = self.traj_df.iloc[j, 1]
    #         y = self.traj_df.iloc[j, 2]
    #         vehicle_id = self.traj_df.iloc[j, 0]
    #         time = self.traj_df.iloc[j, 6]
    #         # print(f"time: {time}")
    #         # if vehicle_id == 46:
    #         #     print('Vehicle ID is 0')
    #         cell_index = self.find_cell(x, y, pneuma_cells)
    #         # print(cell_index)
    #         if cell_index is not None:

    #             if math.isnan(time) and time != 0:
    #                 continue
    #             minute = int(time //  60)
    #             # if minute > 0:
    #             # print("Minute: ", minute)
    #             pneuma_cells[cell_index].add_vehicle(minute, vehicle_id)
    #         # else:
    #         #     print(f'Vehicle at {x}, {y} not in any cell')
    #     for cell in pneuma_cells:
    #         print(cell.vehicle_IDs_each_minute)
    def generate_distribution(self, output_name='output.txt'):
        total_time = float(self.get_max_time())
        total_mins = (total_time // 15) + 1

        # Convert the lat/lon positions to UTM
        self.convert_lat_lon_to_xy()

        min_x, max_x, min_y, max_y = self.get_corner_points()

        # Create a grid of cells
        cells = self.create_cells_from_corners(min_x, max_x, min_y, max_y)
        print("starting vehicle count")

        # Create a list of PneumaCell objects
        pneuma_cells = [PneumaCell(*cell, int(total_mins)) for cell in cells]

        # Calculate the cell index for each row in the DataFrame
        self.traj_df['cell_index'] = self.traj_df.apply(lambda row: self.find_cell(row['lon'], row['lat'], pneuma_cells), axis=1)

        # Filter out rows where time is NaN or 0, and cell_index is not NaN
        valid_rows = self.traj_df[~self.traj_df['time'].isna() & ~self.traj_df['cell_index'].isna()]

        # Convert cell_index to int
        valid_rows['cell_index'] = valid_rows['cell_index'].astype(int)

        # Calculate the minute for each valid row
        valid_rows['minute'] = (valid_rows['time'] // 15).astype(int)

        # Add each vehicle to its corresponding cell
        valid_rows.apply(lambda row: pneuma_cells[int(row['cell_index'])].add_vehicle(int(row['minute']), int(row['track_id'])), axis=1)

        for cell in pneuma_cells:
            print(cell.vehicle_IDs_each_minute)
        
    # Define the directory
        dir_name = 'pneuma data/vehicle distributions/'

        # Check if the directory exists, if not, create it
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        # self.plot_cells_and_positions(pneuma_cells)
        # Write data to a file
        with open(os.path.join(dir_name, output_name), 'w+') as f:
            # header
            f.write('Max time: ' + str(total_time) + '\n')
            f.write('Cell: [x1, y1, x2, y2]: [vehicles per 15 seconds]\n')
            for cell in pneuma_cells:
                print(cell.get_area())
                f.write(str(cell) + '\n')
        
        return pneuma_cells

    def get_max_time(self):
        return self.traj_df.iloc[:, 6].max()
        
        
  

if __name__ == '__main__':
    folder_path = "C:/Users/Alex/Documents/Drones_Testbed/pneuma data"
    # get paths to all CSV files in folder
    files = [f"{folder_path}/{file}" for file in os.listdir(folder_path) if file.endswith('.csv')]
    
    for i, file in enumerate(files):
        if i < 2: continue
        print(file)
        # extract name of file without .csv
        name = file.split("/")[-1].split(".")[0]
        print(name)
        name = name + "_15seconds"
        pnuema_experiments = PnuemaExperimentSetup(file)
        dist = pnuema_experiments.generate_distribution(name + ".txt")
        print(dist)
        
   

    # for cell in dist:
    #     print(cell)
    

