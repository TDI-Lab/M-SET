"""
Workflow executed from here.
"""

# from cdca.src import Basic_Collision_Avoidance, Dependency_Collision_Avoidance, Input_Parser, Swarm_Control
import configparser
from random import randint
from cdca.src.Potential_Fields_Collision_Avoidance import Potential_Fields_Collision_Avoidance
from cdca.src.Input_Parser import Input_Parser
from cdca.src.Swarm_Control import Swarm_Control
from cdca.src.Dependency_Collision_Avoidance import Dependency_Collision_Avoidance
from cdca.src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
from path_generation.ConfigManager import ConfigManager
# from path_generation.experiments.generate_missions import create_random_sensing_missions
# from cdca.src import *
from path_generation.PathGenerator import PathGenerator
import csv
import json
import os
import os.path



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

    

def write_results_to_csv(data, config):

    # Now you can access the values in the config file like this:
    mission_name = config.config.get('global', 'MissionName')
    n_drones = config.config.get('global', 'NumberOfDrones')

    results_path = 'path_generation/PlanGeneration/datasets/'+mission_name+'/results.csv'

    header_exists = os.path.isfile(results_path)
    
    # Write the plans and the results to the CSV file
    with open(results_path, 'a', newline='') as f:
        writer = csv.writer(f)
         # Check if the header has already been written

        # Write the header if it doesn't exist
        if not header_exists:
            writer.writerow(['Strategy', 'n_drones' 'Plan', 'Results'])
        
        # Write the data
        for strategy, plans in data['plans'].items():
            results = data['results'][strategy]
            # Join the plans into a single string
            plans_str = ', '.join(map(str, plans))
            writer.writerow([strategy, n_drones, plans_str, results])
        
        # Add a blank line for readability
        writer.writerow([])

def create_new_random_sensing_mission(size_n, size_m):
    mission_name = f"{size_n}x{size_m}_random.csv"
    rows = ["type,id,x,y,z,value\n"]
    #  Create sensing cells
    cell_id = 0
    for i in range(1, size_n + 1):
        for j in range(1, size_m + 1):
            sensing_value = randint(0, 10)
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

            
def experiment_iteration(n_drones):
    raw = False
    #  Hello :)  There are two steps to running the path generator (once the config is set up)
    #  First, instantiate the PathGenerator object
    pg = PathGenerator()
    #  Then, call PathGenerator.generate_paths.  For CD/CA purposes, you want raw=False (so nothing)
    plans = pg.generate_paths(raw=raw)
    if raw:
        for plan in plans:
            print(plan)
       

    else:
        print("PLANS: ",plans)

        for plan, path in plans.items():
            print(f"plan: {plan}, path: {path}")
        if n_drones == 1:
            plans = [plans]
        input_p = Input_Parser(plans)
        parsed_plans = input_p.parsed_input
        
        print("")
        for plan in parsed_plans:
            print(plan)
        print("")


        swarm_controller = Swarm_Control(parsed_plans, Potential_Fields_Collision_Avoidance(visualise=True))
        swarm_controller.visualise_swarm()
        result = swarm_controller.get_offline_collision_stats()
        plans = swarm_controller.plans

        swarm_controller.detect_potential_collisions()
        plans2 = swarm_controller.plans
        result2 = swarm_controller.get_offline_collision_stats()
        swarm_controller.visualise_swarm()

        swarm_controller2 = Swarm_Control(parsed_plans, Basic_Collision_Avoidance())
        swarm_controller2.detect_potential_collisions()
        plans3 = swarm_controller2.plans
        result3 = swarm_controller2.get_offline_collision_stats()
        swarm_controller2.visualise_swarm()

        return {
        'plans': {
            'no_ca': plans,
            'pf_ca': plans2,
            'basic_ca': plans3,
        },
        'results': {
            'no_ca': result,
            'pf_ca': result2,
            'basic_ca': result3,
        }
    }

if __name__ == '__main__':

    # a list of n m for each experiment grid
    experiment_sizes = [[2,3], [4,4]]

    config = Config('drone_sense.properties')

    n_iterations = 5
    drones = [1,2,3,4]

    for i in range(len(experiment_sizes)):
        abs_path = os.path.abspath('.')
        config.config.set('global', 'MissionName', f"random_{experiment_sizes[i][0]}x{experiment_sizes[i][1]}")

        config.config.set('global', 'MissionFile', f"{abs_path}/examples/{experiment_sizes[i][0]}x{experiment_sizes[i][1]}_random.csv")
        

        for _ in range(n_iterations):
            for n_drones in drones:
                config.config.set('global', 'NumberOfDrones', f"{n_drones}")

                with open(config.config_file_path, 'w') as configfile:
                    config.config.write(configfile)
                create_new_random_sensing_mission(experiment_sizes[i][0], experiment_sizes[i][1])

                data = experiment_iteration(n_drones)
                write_results_to_csv(data, config)