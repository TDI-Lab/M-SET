"""
Workflow executed from here.
"""

# from cdca.src import Basic_Collision_Avoidance, Dependency_Collision_Avoidance, Input_Parser, Swarm_Control
from cdca.src.Potential_Fields_Collision_Avoidance import Potential_Fields_Collision_Avoidance
from cdca.src.Input_Parser import Input_Parser
from cdca.src.Swarm_Control import Swarm_Control
from cdca.src.Dependency_Collision_Avoidance import Dependency_Collision_Avoidance
from cdca.src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
# from cdca.src import *
from path_generation.PathGenerator import PathGenerator
# from cdca.src import *

def main():
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
        swarm_controller = Swarm_Control(parsed_plans, Potential_Fields_Collision_Avoidance(visualise=True))
        swarm_controller.visualise_swarm()

        print("Collisions with no collision avoidance: ")
        result = swarm_controller.get_offline_collision_stats()

        swarm_controller.detect_potential_collisions()
        swarm_controller.visualise_swarm()

        print("Collisions with Potential Fields collision avoidance: ")
        result = swarm_controller.get_offline_collision_stats()
        swarm_controller2 = Swarm_Control(parsed_plans, Basic_Collision_Avoidance())
        # swarm_controller.visualise_swarm()
        print("Collisions with Basic collision avoidance: ")
        result = swarm_controller2.get_offline_collision_stats()

        print("final plan: \n", swarm_controller.plans)
if __name__ == '__main__':
    main()
