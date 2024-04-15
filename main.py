"""
Workflow executed from here.
"""

# from cdca.src import Basic_Collision_Avoidance, Dependency_Collision_Avoidance, Input_Parser, Swarm_Control
import configparser
from cdca.src.Potential_Fields_Collision_Avoidance import Potential_Fields_Collision_Avoidance
from cdca.src.Input_Parser import Input_Parser
from cdca.src.Swarm_Control import Swarm_Control
from cdca.src.Dependency_Collision_Avoidance import Dependency_Collision_Avoidance
from cdca.src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
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



def experiment_iteration():
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