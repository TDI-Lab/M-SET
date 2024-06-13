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

def apply_cdca(parsed_plans, cdca_type):
    if cdca_type == "basic":
        swarm_controller = Swarm_Control(parsed_plans, Basic_Collision_Avoidance())
    elif cdca_type == "pf":
        swarm_controller = Swarm_Control(parsed_plans, Potential_Fields_Collision_Avoidance())
    #print('***********************parsed_plans: ', parsed_plans)
    swarm_controller.detect_potential_collisions()
    new_pl = swarm_controller.plans
    #print('*********************new plans: ', new_pl)
    return new_pl

if __name__ == "__main__":
    """
    os.chdir(original_cwd)
    output_file = "pf_evaluation_paths.txt"
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

    #parsed_plans = [[[[0.0, 0.0], 1], [[1.0, 1.0], 25], [[2.0, 1.0], 3], [[1.0, 2.0], 25], [[0.0, 0.0], 1]], [[[4.0, 0.0], 1], [[3.0, 1.0], 38], [[2.0, 1.0], 7], [[3.0, 2.0], 7], [[4.0, 0.0], 1]], [[[0.0, 3.0], 1], [[1.0, 2.0], 22], [[2.0, 2.0], 8], [[1.0, 1.0], 22], [[0.0, 3.0], 1]], [[[4.0, 3.0], 1], [[3.0, 2.0], 6], [[3.0, 1.0], 30], [[2.0, 2.0], 18], [[4.0, 3.0], 1]]]

    file.write(str(parsed_plans))
    file.write("\n")

    print("Number of drones in path %s" % len(parsed_plans))

    # Generate the 1, 2, 3, 4 drone cdca paths
    print("")
    for n_drones in range(4,0,-1):
        print("%s drones" % n_drones)
        cdca_plans = apply_cdca(parsed_plans[:n_drones])
        print(cdca_plans)

        file.write(str(cdca_plans))
        file.write("\n")

        #swarm_controller.plans =  execute these plans with hardware/simulation
        hardware_main(cdca_plans, raw=True, input_mode="cdca")
        print("")

    """

    parsed_plans = [[[[-0.8299, -0.47], 1], [[-0.5533, -0.235], 1], [[-0.5533, 0.235], 1], [[0.0, -0.235], 2], [[-0.8299, -0.47], 1]], [[[0.8299, -0.47], 1], [[0.5533, 0.235], 1], [[0.0, -0.235], 2], [[0.8299, -0.47], 1]], [[[0.8299, 0.47], 1], [[0.5533, 0.235], 1], [[0.0, -0.235], 2], [[0.8299, 0.47], 1]], [[[-0.8299, 0.47], 1], [[-0.5533, 0.235], 1], [[-0.5533, -0.235], 1], [[0.0, -0.235], 2], [[-0.8299, 0.47], 1]]]
    parsed_plans = parsed_plans[:1]
    print(parsed_plans)
    print("")
    cdca_plans = apply_cdca(parsed_plans, "pf")
    print(cdca_plans)