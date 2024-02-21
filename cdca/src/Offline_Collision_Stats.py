from .Basic_Collision_Avoidance import Basic_Collision_Avoidance

class Offline_Collision_Stats(Basic_Collision_Avoidance):
# A class to calculate collision stats offline.
  def __init__(self, drones):
    # Initialise the Basic Collection Detction/Avoidance strategy with the 
    # 'only_collision_detection' flag set to true.
    self.only_collision_detection = True
    self.number_of_collisions = 0
    self.detect_potential_collisions(drones)

  def get_number_of_collisions(self):
    return self.number_of_collisions