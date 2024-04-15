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
# from path_generation.experiments.generate_missions import create_random_sensing_missions
# from cdca.src import *
from path_generation.PathGenerator import PathGenerator
import csv
import json
import os

def write_results_to_csv(data, config_file_path='drone_sense.properties'):
   # Load the config file
    config = configparser.ConfigParser()
    config.read(config_file_path)

    # Get the model name from the config file
    model_name = config.get('DEFAULT', 'ModelName', fallback='default')

    # Check if the header has already been written
    header_exists = os.path.isfile('path_generation/PlanGeneration/datasets/'+model_name+'_results.csv')

    # Write the plans and the results to the CSV file
    with open('path_generation/PlanGeneration/datasets/'+model_name+'_results.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        
        # Write the header if it doesn't exist
        if not header_exists:
            writer.writerow(['Strategy', 'Plan', 'Results'])
        
        # Write the data
        for strategy, plans in data['plans'].items():
            results = data['results'][strategy]
            # Join the plans into a single string
            plans_str = ', '.join(map(str, plans))
            writer.writerow([strategy, plans_str, results])
        
        # Add a blank line for readability
        writer.writerow([])

def create_new_random_sensing_mission(size=3):
    sizes = [i for i in range(2, 13)]
    mission_name = f"{size}x{size}_random.csv"
    rows = ["type,id,x,y,z,value\n"]
    #  Create sensing cells
    cell_id = 0
    for i in range(1, size + 1):
        for j in range(1, size + 1):
            sensing_value = randint(0, 10)
            new_row = f"SENSE,{cell_id},{i},{j},1,{sensing_value}\n"
            rows.append(new_row)
            cell_id += 1
    rows.append(f"BASE,0,0,0,0,0\n")
    rows.append(f"BASE,1,{size + 1},0,0,0\n")
    rows.append(f"BASE,2,0,{size + 1},0,0\n")
    rows.append(f"BASE,3,{size + 1},{size + 1},0,0\n")
    with open(f"examples/{mission_name}", "w") as file:
        for row in rows:
            file.write(row)

def experiment_iteration():
    raw = False
    #  Hello :)  There are two steps to running the path generator (once the config is set up)
    #  First, instantiate the PathGenerator object
    create_new_random_sensing_mission()
    pg = PathGenerator()
    #  Then, call PathGenerator.generate_paths.  For CD/CA purposes, you want raw=False (so nothing)
    plans = pg.generate_paths(raw=raw)
    if raw:
        for plan in plans:
            print(plan)
       

    else:
        for plan, path in plans.items():
            print(f"plan: {plan}, path: {path}")

        input_p = Input_Parser(plans)
        parsed_plans = input_p.parsed_input
        
        print("")
        for plan in parsed_plans:
            print(plan)
        print("")


        swarm_controller = Swarm_Control(parsed_plans, Potential_Fields_Collision_Avoidance())
        result = swarm_controller.get_offline_collision_stats()
        plans = swarm_controller.plans

        swarm_controller.detect_potential_collisions()
        plans2 = swarm_controller.plans
        result2 = swarm_controller.get_offline_collision_stats()

        swarm_controller2 = Swarm_Control(parsed_plans, Basic_Collision_Avoidance())
        swarm_controller2.detect_potential_collisions()
        plans3 = swarm_controller2.plans
        result3 = swarm_controller2.get_offline_collision_stats()

        del swarm_controller
        del swarm_controller2
        del pg

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
    for i in range(3):
        data = experiment_iteration()
        write_results_to_csv(data)