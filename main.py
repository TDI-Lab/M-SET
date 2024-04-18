"""
Workflow executed from here.
"""


import copy
from cdca.src.Input_Parser import Input_Parser
from cdca.src.Potential_Fields_Collision_Avoidance import Potential_Fields_Collision_Avoidance
from cdca.src.Swarm_Control import Swarm_Control
from cdca.src.Dependency_Collision_Avoidance import Dependency_Collision_Avoidance
from cdca.src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
from experiments.MeasureSensing import MeasureSensing
from experiments.VisualiseData import VisualiseData

from path_generation.PathGenerator import PathGenerator
import csv
import json
import os
import os.path
import configparser
from random import randint

class Config:
    def __init__(self, config_file_path='drone_sense.properties'):
        self.config_file_path = config_file_path
        # Check if the config file exists
        if not os.path.isfile(config_file_path):
            print(f"Config file '{config_file_path}' not found.")
            return

        self.config = configparser.ConfigParser()
        self.config.optionxform = str  # preserve case for options

        self.config.read(config_file_path)

    
def write_results_to_csv(data, config, greedy=False):

    # Now you can access the values in the config file like this:
    mission_name = config.config.get('global', 'MissionName')
    if greedy:
        mission_name += '_greedy'
    n_drones = config.config.get('global', 'NumberOfDrones')

    results_path = 'experiments/results/'+mission_name+'_results.csv'

    header_exists = os.path.isfile(results_path)
    
    # Write the plans and the results to the CSV file
    with open(results_path, 'a', newline='') as f:
        writer = csv.writer(f)
         # Check if the header has already been written

        # Write the header if it doesn't exist
        if not header_exists:
            writer.writerow(['Strategy', 'n_drones', 'Plan', 'Sensing Mismatch %','Total Collisions', 'Cross Collisions', 'Parallel_Collisions', 'Cell Occupied Collisions', 'Total Flights Distance', 'Total Collision Distance', 'Risk of Collision', 'Total Duration of Flights', 'Number of Flights', 'Average Collisions per Flight'])
        
        # Write the data
        for strategy, _ in data['plans'].items():
            collisions = data['results'][strategy]
            sensing_mismatch = data['sensing mismatch'][strategy]
            plans = data['plans'][strategy]
            # Join the plans into a single string
            writer.writerow([strategy, n_drones, plans, sensing_mismatch, collisions['number_of_collisions'], collisions['number_of_cross_collisions'],collisions['number_of_parallel_collisions'], collisions['number_of_dest_occupied_collisions'], collisions['total_flights_distance'], collisions['total_collision_distance'], collisions['risk_of_collision'], collisions['total_duration_of_flights'], collisions['number_of_flights'], collisions['average_collisions_per_flight']])
        
        # Add a blank line for readability
        writer.writerow([])

def create_new_random_sensing_mission(size_n, size_m):
    mission_name = f"{size_n}x{size_m}_random.csv"
    rows = ["type,id,x,y,z,value\n"]
    #  Create sensing cells
    cell_id = 0
    for i in range(1, size_n + 1):
        for j in range(1, size_m + 1):
            sensing_value = randint(1, 10)
            new_row = f"SENSE,{cell_id},{i},{j},1,{sensing_value}\n"
            rows.append(new_row)
            cell_id += 1
    rows.append(f"BASE,0,0,0,0,0\n")
    rows.append(f"BASE,1,{size_n + 1},0,0,0\n")
    rows.append(f"BASE,2,0,{size_m + 1},0,0\n")
    rows.append(f"BASE,3,{size_n + 1},{size_m + 1},0,0\n")
    with open(f"examples/{mission_name}", "w") as file:
        for row in rows:
            file.write(row)

    return mission_name

            
def experiment_1and2_iteration(n_drones, mission_name):

    pg = PathGenerator()
    plans = pg.generate_paths()

    for plant, path in plans.items():
        print(f"plan: {plant}, path: {path}")

    # if n_drones == 1:
    #     plans = [plans]

    input_p = Input_Parser(plans)
    parsed_plans = input_p.parsed_input
    
    print("")
    for planx in parsed_plans:
        print(planx)
    print("")

    sensing = MeasureSensing(f"examples/{mission_name}")

    swarm_controller = Swarm_Control(parsed_plans, Potential_Fields_Collision_Avoidance())
    swarm_controller2 = Swarm_Control(parsed_plans, Potential_Fields_Collision_Avoidance())
    swarm_controller3 = Swarm_Control(parsed_plans, Basic_Collision_Avoidance())

    # priority_map = swarm_controller.determine_priority(parsed_plans)
    # swarm_controller4 = Swarm_Control(parsed_plans, Basic_Collision_Avoidance(priority_map=priority_map))

    swarm_controller2.detect_potential_collisions()
    swarm_controller3.detect_potential_collisions()
    # swarm_controller2.visualise_swarm()
    # swarm_controller4.detect_potential_collisions()


    no_ca_plans =  [[plan1 for plan1 in drone.plan] for drone in swarm_controller.drones]
    pf_plans = [[plan2 for plan2 in drone.plan] for drone in swarm_controller2.drones]
    basic_plans = [[plan3 for plan3 in drone.plan] for drone in swarm_controller3.drones]
    # priority_plans = [[plan4 for plan4 in drone.plan] for drone in swarm_controller4.drones]

    return {
    'plans': {
        'no_ca':no_ca_plans,
        'pf_ca': pf_plans,
        'basic_ca':basic_plans,
        # 'priority_ca': priority_plans,
    },
    'results': {
        'no_ca': swarm_controller.get_offline_collision_stats(),
        'pf_ca': swarm_controller2.get_offline_collision_stats(),
        'basic_ca': swarm_controller3.get_offline_collision_stats(),
        # 'priority_ca': swarm_controller4.get_offline_collision_stats(),
    },
    'sensing mismatch': {
        'no_ca': sensing.measure_sensing(no_ca_plans),
        'pf_ca': sensing.measure_sensing(pf_plans),
        'basic_ca': sensing.measure_sensing(basic_plans),
        # 'priority_ca': sensing.measure_sensing(priority_plans),
    }
}
def run_experiment_1and2():
     # a list of n m for each experiment grid
    experiment_sizes = [[2,3],[3,3],[4,4],[5,5],[6,6],[7,7],[8,8],[9,9],[10,10]]

    config = Config('drone_sense.properties')

    n_iterations = 5
    drones = [1,2,3,4,5,6,7,8]
    # drones = [5,6,7,8]

    for i in range(len(experiment_sizes)):
        abs_path = os.path.abspath('.')
        config.config.set('global', 'MissionName', f"random_{experiment_sizes[i][0]}x{experiment_sizes[i][1]}")

        config.config.set('global', 'MissionFile', f"{abs_path}/examples/{experiment_sizes[i][0]}x{experiment_sizes[i][1]}_random.csv")
        

        for _ in range(n_iterations):
            for n_drones in drones:
                config.config.set('global', 'NumberOfDrones', f"{n_drones}")

                with open(config.config_file_path, 'w') as configfile:
                    config.config.write(configfile)
                mission_name = create_new_random_sensing_mission(experiment_sizes[i][0], experiment_sizes[i][1])

                data = experiment_1and2_iteration(n_drones, mission_name)
                write_results_to_csv(data, config)
                


if __name__ == '__main__':
    run_experiment_1and2()

    data_paths = ['experiments/results/random_2x3_results.csv','experiments/results/random_3x3_results.csv','experiments/results/random_4x4_results.csv','experiments/results/random_5x5_results.csv', 
                  'experiments/results/random_6x6_results.csv', 'experiments/results/random_7x7_results.csv', 'experiments/results/random_8x8_results.csv']
                #   'experiments/results/random_9x9_results.csv', 'experiments/results/random_10x10_results.csv']
    data_paths = [ 'experiments/results/random_2x3_results.csv']
    # map_name = "2x3"
    # Visualise the data
    # for path in data_paths:
        # print(path)
        #extract the 5x5 from the path
        # map_name = path.split('/')[-1].split('_')[1]
        # print(map_name)
    # vd = VisualiseData(data_paths, 'experiments/results/')
    # vd.plotNumAgentsVsSensingAccuracy(map_name)
    # vd.plotNumAgentsVsCollisions()

   