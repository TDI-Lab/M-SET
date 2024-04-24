#     Test 1 drone, 2 drones, 3 drones, and 4 drones using EPOS with collision avoidance (6 cells); 
import os
original_cwd = os.getcwd()

from path_generation.PathGenerator import PathGenerator
from cdca.src.Input_Parser import Input_Parser
from cdca.src.Swarm_Control import Swarm_Control
from cdca.src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
from cdca.src.Potential_Fields_Collision_Avoidance import Potential_Fields_Collision_Avoidance
from Hardware.cdca_epos_executor import main as hardware_main

def generate_epos():
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
            print(plan, path)
    #print(pg.get_coordinate_position_results())

    return plans

def apply_cdca(parsed_plans):
    swarm_controller = Swarm_Control(parsed_plans, Basic_Collision_Avoidance())
    swarm_controller.detect_potential_collisions()

    return swarm_controller.plans

if __name__ == "__main__":
    os.chdir(original_cwd)
    output_file = "evaluation_paths.txt"
    try:
        file = open("Hardware/Experiments/%s" % output_file, "w")
    except:
        print("Experiment output file not found")

    plans = generate_epos()

    epos_path = [path for plan,path in plans.items()]
    file.write(str(epos_path))
    file.write("\n")

    print("")

    input_p = Input_Parser(plans)
    parsed_plans = input_p.parsed_input

    file.write(str(parsed_plans))
    file.write("\n")

    print("Number of drones in path %s" % len(parsed_plans))

    # Generate the 1, 2, 3, 4 drone cdca paths
    print("")
    for n_drones in range(5,1,-1):
        print("%s drones" % n_drones)
        cdca_plans = apply_cdca(parsed_plans[:n_drones])
        print(cdca_plans)

        file.write(str(cdca_plans))
        file.write("\n")

        #swarm_controller.plans =  execute these plans with hardware/simulation
        hardware_main(cdca_plans, raw=True, input_mode="cdca")
        print("")