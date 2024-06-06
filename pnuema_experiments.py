import copy
import csv
import math
import os
import random
import ast

from cdca.src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
from cdca.src.Input_Parser import Input_Parser
from cdca.src.Potential_Fields_Collision_Avoidance import Potential_Fields_Collision_Avoidance
from cdca.src.Swarm_Constants import TIME_STEP
from cdca.src.Swarm_Control import Swarm_Control
from experiments.MeasureSensing import MeasureSensing
from main import Config
from path_generation.PathGenerator import PathGenerator
from experiments.pneuma.pnuema_setup import PneumaCell

class PneumaExperiment:
    def __init__(self, filename=None, synthesise=False):
        if synthesise:
            self.synthesiste_data()
        else:
            self.filename = filename
            self.load_data(True)

    def load_data(self, is_random=False):
        with open(self.filename, 'r') as f:
            self.data = f.readlines()

        # data is Cell: [x1, y1, x2, y2]: [vehicles per minute] per line
        self.cells = []
        for i, line in enumerate(self.data):
            if i == 0:
                self.max_time = int(math.ceil(float(line.split(':')[-1].strip())))
                continue
            if i == 1: continue
            line = line.strip().split(':')
            cell = ast.literal_eval(line[0])
            data = line[1].split('[')[1].split(']')[0].split(',')
            data = [int(x) for x in data]
            if is_random:
                self.cells.append( PneumaCell(cell[0], cell[1], cell[2], cell[3] ,self.max_time,[random.randint(0, 10) for _ in range(self.max_time)], i-2))

            else:
                self.cells.append( PneumaCell(cell[0], cell[1], cell[2], cell[3] ,self.max_time,data, i-2))


        print("asd")
    def print_cell_distances(self):
        for cell in self.cells:
            for other_cell in self.cells:
                print(f"Distance between {cell.id} and {other_cell.id}: {cell.get_distance_from_other_cell(other_cell)}")
    # Generates random data but to ensure that the data is not too similar, the cells are given a priority
    def synthesiste_data(self):
        synth = {
            ((2248579.713448535, 2730009.104624533, 2248799.281194936, 2730280.2185926433)),
            ((2248579.713448535, 2730280.2185926433, 2248799.281194936, 2730551.332560753)),
            ((2248579.713448535, 2730551.332560753, 2248799.281194936, 2730822.446528863)),
            ((2248799.281194936, 2730009.104624533, 2249018.848941337, 2730280.2185926433)),
            ((2248799.281194936, 2730280.2185926433, 2249018.848941337, 2730551.332560753)),
            ((2248799.281194936, 2730551.332560753, 2249018.848941337, 2730822.446528863))
        }
        self.cells = {}
        total_time = 30
        for priority, cell in enumerate(synth, start=1):
            min_x, max_x, min_y, max_y = self.put_grid_at_origin(cell[0], cell[2], cell[1], cell[3])
            self.cells[priority] = PneumaCell(min_x, max_x, min_y, max_y,total_time,[random.randint(0, 10) * priority for _ in range(total_time)],priority)
                

        print(self.cells)



    def get_cell_priority(self):

        # generate average number of vehicles per cell and sort
        cell_priority = {}
        for cell in self.cells:
            cell_priority[cell] = sum(cell.num_vehicles_per_minute) / len(cell.num_vehicles_per_minute)

        cell_priority = sorted(cell_priority.items(), key=lambda x: x[1], reverse=True)
        return cell_priority

    def get_ratio_of_times(self, cells):
        total_vehicles = 0
        for cell in cells:
            total_vehicles += sum(cell.num_vehicles_per_minute)
        self.total_vehicles = total_vehicles
        ratio_vehicles = {}
        for cell in cells:
            ratio_vehicles[cell.id] = sum(cell.num_vehicles_per_minute) / total_vehicles
        total = sum(ratio_vehicles.values())

        ratio_time = {}
        for i, cell_vehicles in enumerate(ratio_vehicles.values()):
            ratio_time[i] = cell_vehicles * self.max_time

        return ratio_time
  

    
    
    def get_corner_points(self):
        min_x = min([cell.cell[0] for cell in self.cells])
        max_x = max([cell.cell[2] for cell in self.cells])
        min_y = min([cell.cell[1] for cell in self.cells])
        max_y = max([cell.cell[3] for cell in self.cells])

        return min_x, max_x, min_y, max_y
    
    def generate_sensing_mission(self):
        # get cell priority
        # /cell_priority = self.get_cell_priority()

        time_ratio = self.get_ratio_of_times(self.cells)

        # generate sensing mission
        mission = ["type,id,x,y,z,value\n"]
        for i, cell in enumerate(self.cells):
            centroid_x, centroid_y = cell.get_centroid()
            print("centroid", centroid_x, " ", centroid_y)
            mission.append(f"SENSE,{i},{centroid_x},{centroid_y},1,{time_ratio[i]}\n")

        min_x, max_x, min_y, max_y = self.get_corner_points()
        self.print_cell_distances()
        mission.append(f"BASE,0,{min_x},{min_y},0,0\n")
        mission.append(f"BASE,1,{max_x},{min_y},0,0\n")
        mission.append(f"BASE,2,{min_x},{max_y},0,0\n")
        mission.append(f"BASE,3,{max_x},{max_y},0,0\n")

        with open('examples/new_pneuma_points.csv', 'w') as f:
            for line in mission:
                f.write(line)

        #visulise the mission
        # self.visualise_mission(mission)

    def visualise_mission(self, mission):
        import matplotlib.pyplot as plt
        for i, line in enumerate(mission):
            if i == 0: continue
            line = line.split(',')
            if line[0] == "SENSE":
                plt.scatter(float(line[2]), float(line[3]), c='blue')
            else:
                plt.scatter(float(line[2]), float(line[3]), c='red')
        plt.show()

    # Find the cell that a point is in
    def find_cell(self, x, y, cells):
        i = 0
        for cell in cells:
            if cell.is_in_cell(x, y):
                return i
            i+=1
        return None
    
    def put_grid_at_origin(self, min_x, max_x, min_y, max_y):
        width = max_x - min_x
        height = max_y - min_y
    
        # New grid starts at (0, 0) and has the same dimensions
        new_min_x = 0
        new_max_x = new_min_x + width
        new_min_y = 0
        new_max_y = new_min_y + height
    
        return new_min_x, new_max_x, new_min_y, new_max_y  
    
    def find_majority_cell(self,path, cells):
        # Initialize a dictionary to store the time spent in each cell
        time_in_cells = {cell: 0 for cell in cells}

        # Iterate over the path
        for [x,y] in path:
            # Get the cell for the current point
            cell = self.find_cell(x, y, cells)
            if cell is not None:
                # Add the time spent at the current point to the total time for this cell
                time_in_cells[self.cells[cell]] += TIME_STEP

        # Find the cell with the maximum time
        majority_cell = max(time_in_cells, key=time_in_cells.get)

        return majority_cell
    
    def calculate_observed_vehicles(self, swarm_controller, total_time):
        observed_vehicles = [0] * total_time
        observed_vehciles_per_cell = [0] * len(self.cells)
        for minute in range(total_time):
            discreet_positions = swarm_controller.discretise_flight_paths(swarm_controller.drones, from_time=(minute/TIME_STEP)*60, to_time=((minute+1)/TIME_STEP)*60)
            if discreet_positions is not None:
                for drone_positions in discreet_positions:
                    if drone_positions is None:
                        continue
                    if all(position is None for position in drone_positions):
                        continue
                    cell = self.find_majority_cell(drone_positions, self.cells)
                    observed_vehicles[minute] += cell.num_vehicles_per_minute[minute]
                    observed_vehciles_per_cell[cell.id] += cell.num_vehicles_per_minute[minute]

        return observed_vehicles, observed_vehciles_per_cell

    def experiment_iteration(self,n_drones, mission_name, total_time=30):

        pg = PathGenerator()
        plans = pg.generate_paths()

        for plant, path in plans.items():
            print(f"plan: {plant}, path: {path}")


        input_p = Input_Parser(plans)
        parsed_plans = input_p.parsed_input
        
        print("")
        for planx in parsed_plans:
            print(planx)
        print("")

        sensing = MeasureSensing(f"examples/{mission_name}")

        swarm_controller = Swarm_Control(copy.deepcopy(parsed_plans), Basic_Collision_Avoidance())
        swarm_controller2 = Swarm_Control(copy.deepcopy(parsed_plans), Potential_Fields_Collision_Avoidance(visualise=False))
        swarm_controller3 = Swarm_Control(copy.deepcopy(parsed_plans), Basic_Collision_Avoidance())

        print("Starting PF CA")
        swarm_controller2.detect_potential_collisions()
        print("Starting Basic CA")

        swarm_controller3.detect_potential_collisions()



        no_ca_plans =  swarm_controller.plans
        pf_plans = swarm_controller2.plans
        basic_plans = swarm_controller3.plans

        total_mins = self.max_time // 60
    

        return {
        'plans': {
            'no_ca':no_ca_plans,
            'pf_ca': pf_plans,
            'basic_ca':basic_plans,
        },
        'results': {
            'no_ca': swarm_controller.get_offline_collision_stats(),
            'pf_ca': swarm_controller2.get_offline_collision_stats(),
            'basic_ca': swarm_controller3.get_offline_collision_stats(),
        },
        'sensing mismatch': {
            'no_ca': sensing.measure_sensing(no_ca_plans),
            'pf_ca': sensing.measure_sensing(pf_plans),
            'basic_ca': sensing.measure_sensing(basic_plans),
        },
        'observed_vehicles': {
            'no_ca':self.calculate_observed_vehicles(swarm_controller,total_mins ),
            'pf_ca': self.calculate_observed_vehicles(swarm_controller2, total_mins),
            'basic_ca':self.calculate_observed_vehicles(swarm_controller3, total_mins)
            }

    }

    def run_pneuma_experiment(self):

        config = Config('drone_sense.properties')
        abs_path = os.path.abspath('.')

        mission = 'new_pneuma_points'

        #set drone properties
        config.config.set('drone', 'BatteryCapacity', f"275000")
        config.config.set('drone', 'BodyMass', f"1.07")
        config.config.set('drone', 'BatteryMass', f"0.31")
        config.config.set('drone', 'PowerEfficiency', f"0.8")
        
        config.config.set('global', 'MissionName', mission)

        config.config.set('global', 'MissionFile', f"{abs_path}/examples/{mission}.csv")
        
        
        n_iterations = 200
        drones = [4]


        for _ in range(n_iterations):
            for n_drones in drones:
                config.config.set('global', 'NumberOfDrones', f"{n_drones}")

                with open(config.config_file_path, 'w') as configfile:
                    config.config.write(configfile)
                self.generate_sensing_mission()
                data = self.experiment_iteration(n_drones, mission+".csv")
                self.write_results_to_csv(data, config, experiment_name='pneuma')
            

    def write_results_to_csv(self,data, config, experiment_name=None):

        # Now you can access the values in the config file like this:
        mission_name = config.config.get('global', 'MissionName')
        n_drones = config.config.get('global', 'NumberOfDrones')

        if experiment_name is not None:
            #if folder called experiment_name at 'experiments/results/' does not exist, create it
            if not os.path.exists('experiments/results/pnuema/'+experiment_name):
                os.makedirs('experiments/results/pnuema/'+experiment_name)
                experiment_name = experiment_name + '/'
        else:
            experiment_name = ''
        results_path = 'experiments/results/pnuema/'+experiment_name + '/' + mission_name+'_results.csv'

        header_exists = os.path.isfile(results_path)
        
        # Write the plans and the results to the CSV file
        with open(results_path, 'a', newline='') as f:
            writer = csv.writer(f)

            # Write the header if it doesn't exist
            if not header_exists:
                writer.writerow(['Strategy', 'n_drones', 'Plan', 'plan overflow', 'Sensing Mismatch %','UnderSensing %','Oversensing %','Total Collisions', 'Cross Collisions', 'Parallel Collisions', 'Dest. Occupied Collisions', 'Total Flights Distance', 'Total Collision Distance', 'Risk of Collision', 'Total Duration of Flights', 'Total Hover Duration',  'Number of Flights', 'Average Collisions per Flight','Observed Vehicles Per Minute', 'Observed Vehicles Per Cell','Total Observed Vehicles','Total Vehicles'])
            
            # Write the data
            for strategy, _ in data['plans'].items():
                collisions = data['results'][strategy]
                observed_vehicles, observed_vehciles_per_cell = data['observed_vehicles'][strategy]
                sensing_mismatch, undersensing_percentage,  oversensing_percentage  = data['sensing mismatch'][strategy]
                plans = data['plans'][strategy]
                # Truncate plans to a maximum length of 32700 characters
                plans_str = str(plans)
                plans_truncated = plans_str[:32700]
                plans_overflow = plans_str[32700:]
                if len(plans_overflow) > 32700:
                    plans_overflow = "Too long to display"
                    
                # Join the plans into a single string
                writer.writerow([strategy, n_drones, plans_truncated, plans_overflow, sensing_mismatch, undersensing_percentage,  oversensing_percentage, collisions['number_of_collisions'], collisions['number_of_cross_collisions'],collisions['number_of_parallel_collisions'], collisions['number_of_dest_occupied_collisions'], collisions['total_flights_distance'], collisions['total_collision_distance'], collisions['risk_of_collision'], collisions['total_duration_of_flights'],collisions['total_hover_duration'], collisions['number_of_flights'], collisions['average_collisions_per_flight'], observed_vehicles, observed_vehciles_per_cell, sum(observed_vehicles), self.total_vehicles])
            
            # Add a blank line for readability
            writer.writerow([])


if __name__ == '__main__':
    p = PneumaExperiment('output.txt', False)
    print(p.cells)
    p.run_pneuma_experiment()
    