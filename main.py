"""
Workflow executed from here.
"""


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
            writer.writerow(['Strategy', 'n_drones', 'Plan', 'Sensing Mismatch %','Total Collisions', 'Cross Collisions', 'Parallel_Collisions', 'Cell Occupied Collisions'])
        
        # Write the data
        for strategy, _ in data['plans'].items():
            collisions = data['results'][strategy]
            sensing_accuracy = data['sensing mismatch'][strategy]
            plans = data['plans'][strategy]
            # Join the plans into a single string
            writer.writerow([strategy, n_drones, plans, sensing_accuracy, collisions['number_of_collisions'], collisions['number_of_cross_collisions'], collisions['number_of_dest_occupied_collisions']])
        
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

    for plan, path in plans.items():
        print(f"plan: {plan}, path: {path}")

    # if n_drones == 1:
    #     plans = [plans]

    input_p = Input_Parser(plans)
    parsed_plans = input_p.parsed_input
    
    print("")
    for plan in parsed_plans:
        print(plan)
    print("")

    sensing = MeasureSensing(f"examples/{mission_name}")

    swarm_controller = Swarm_Control(parsed_plans, Potential_Fields_Collision_Avoidance()) # No collision avoidance actually being used here
    swarm_controller2 = Swarm_Control(parsed_plans, Potential_Fields_Collision_Avoidance(visualise=False))
    swarm_controller3 = Swarm_Control(parsed_plans, Basic_Collision_Avoidance())

    swarm_controller2.detect_potential_collisions()
    swarm_controller3.detect_potential_collisions()

    plans =  [[plan for plan in drone.plan] for drone in swarm_controller.drones]
    plans2 = [[plan for plan in drone.plan] for drone in swarm_controller2.drones]
    plans3 = [[plan for plan in drone.plan] for drone in swarm_controller3.drones]
    
    print("Plans: ")
    print("No CA: ",plans)
    # print("PF CA: ",plans2)
    print("Basic CA: ",plans3)
    return {
    'plans': {
        'no_ca':plans,
        'pf_ca': plans2,
        'basic_ca':plans3,
    },
    'results': {
        'no_ca': swarm_controller.get_offline_collision_stats(),
        'pf_ca': swarm_controller2.get_offline_collision_stats(),
        'basic_ca': swarm_controller3.get_offline_collision_stats(),
    },
    'sensing mismatch': {
        'no_ca': sensing.measure_sensing(plans),
        'pf_ca': sensing.measure_sensing(plans2),
        'basic_ca': sensing.measure_sensing(plans3),
    }
}
def run_experiment_1and2():
     # a list of n m for each experiment grid
    experiment_sizes = [[2,3], [4,4]]

    config = Config('drone_sense.properties')

    n_iterations = 20
    drones = [5,2,3,4,5,6,7,8]

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
    # run_experiment_1and2()


    # Visualise the data
    vd = VisualiseData('experiments/results/random_2x3_results.csv', 'experiments/results/')
    vd.plotNumAgentsVsSensingAccuracy()
    # vd.plotNumAgentsVsCollisions()

   