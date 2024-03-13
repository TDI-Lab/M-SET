from .Basic_Collision_Avoidance import Basic_Collision_Avoidance
from .Swarm_Constants import *

class Offline_Collision_Stats(Basic_Collision_Avoidance):
# A class to calculate collision stats offline.
  def __init__(self, drones):
    # Initialise the Basic Collection Detction/Avoidance strategy with the 
    # 'only_collision_detection' flag set to true.
    super(Offline_Collision_Stats, self).__init__(True)
    self.detect_potential_collisions(drones)

  def get_number_of_collisions(self):
    return len(self.collisions)
  
  def get_number_of_cross_collisions(self):
    count = 0
    for collision in self.collisions:
      if collision.collision_type == CROSS_COLLISION:
        count += 1
    return count
  
  def get_number_of_parallel_collisions(self):
    count = 0
    for collision in self.collisions:
      if collision.collision_type == PARALLEL_COLLISION:
        count += 1
    return count

  def get_number_of_dest_occupied_collisions(self):
    count = 0
    for collision in self.collisions:
      if collision.collision_type == DEST_OCCUPIED_COLLISION:
        count += 1
    return count
    