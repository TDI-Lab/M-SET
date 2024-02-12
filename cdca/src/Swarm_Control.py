from .Drone import Drone
from .Collision_Strategy import Collision_Strategy
from .Basic_Collision_Avoidance import Basic_Collision_Avoidance
from .Swarm_Constants import COLLISION_AVOIDANCE_LIMIT

class Swarm_Control:
  # Class to control the swarm on the test bed.

  def __init__(self, plans, collision_strategy):
    # Initialise a swarm controller with drone plans and a collision avoidance strategy.
    self.plans = plans
    self.collision_strategy = collision_strategy
    self.initialise_drones()
  
  def initialise_drones(self):
    # Create drone objects based on the plans.
    self.drones = []
    for plan in self.plans:
      new_drone = Drone(plan)
      self.drones.append(new_drone)

  def detect_potential_collisions(self):
    # Detect and fix potential collisions between drones.
    for i in range(COLLISION_AVOIDANCE_LIMIT):
      if (not self.collision_strategy.detect_potential_collisions(self.drones)):
        return

 
  def print_itinerary(self):
    print("Drone plans: ", *self.plans,sep='\n')
    print("\nDrones: \n\n")
    drone_index = 0
    for drone in self.drones:
      print("\nDrone ", drone_index, ": \n")
      drone.print_itinerary()
      drone_index += 1
      print("**********************************")

