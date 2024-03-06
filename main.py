"""
Workflow executed from here.
"""

from path_generation.PathGenerator import PathGenerator
from cdca.src.Input_Parser import Input_Parser
from cdca.src.Swarm_Control import Swarm_Control
from cdca.src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
from cdca.src.Dependency_Collision_Avoidance import Dependency_Collision_Avoidance


def main():
    pg = PathGenerator()
    plans = pg.generate_paths(raw=False)
    for plan, path in plans.items():
        print(plan, path)
    new_plans = []
    
    input_p = Input_Parser(plans)
    parsed_plans = input_p.parsed_input
    
    for plan in parsed_plans:
        print(plan)
    swarm_controller = Swarm_Control(parsed_plans, Basic_Collision_Avoidance())
    swarm_controller.detect_potential_collisions()
    
    #swarm_controller.plans =  execute these plans with hardware/simulation



if __name__ == '__main__':
    main()
