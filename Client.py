from src.Swarm_Control import Swarm_Control
from src.Basic_Collision_Avoidance import Basic_Collision_Avoidance

plans = [[[[1,1], 5], [[9,9], 3]], [[[9,1], 5], [[1,9], 3]]]

swarm_controller = Swarm_Control(plans, Basic_Collision_Avoidance())
swarm_controller.print_itinerary()

swarm_controller.detect_potential_collisions()
swarm_controller.print_itinerary()