from src.Swarm_Control import Swarm_Control
from src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
from src.Input_Parser import Input_Parser
from src.Swarm_Constants import EPOS_TIMESTEP



# example_input = {1: [(1,1), (1,1), (5,5), (5,5)], 
#                  2: [(1,3), (3,1), (2,5), (5,3)], 
#                  3: [(1,9), (7,4), (5,1), (5,1)], 
#                  4: [(9,9), (2,2), (2,2), (5,7)]}

# plans = Input_Parser(example_input).parsed_input

plans = [[[[1,1], 5], [[9,9], 3]], [[[9,1], 5], [[1,9], 3]], [[[1,9], 20], [[1,1], 3]]]

priority_map = Swarm_Control.determine_priority(plans)

swarm_controller = Swarm_Control(plans, Basic_Collision_Avoidance(priority_map = priority_map))

swarm_controller.print_itinerary()
# swarm_controller.get_offline_collision_stats()

swarm_controller.detect_potential_collisions()

# swarm_controller.get_offline_collision_stats()
swarm_controller.print_itinerary()

