from .Basic_Collision_Avoidance import Basic_Collision_Avoidance
from .Swarm_Constants import *

class Offline_Collision_Stats(Basic_Collision_Avoidance):
# A class to calculate collision stats offline.
  def __init__(self, drones):
    # Initialise the Basic Collection Detction/Avoidance strategy with the 
    # 'only_collision_detection' flag set to true.
    super(Offline_Collision_Stats, self).__init__(True)
    self.detect_potential_collisions(drones)
    self.drones =  drones

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

  def get_total_flights_distance(self):
    total_dist = 0
    for drone in self.drones:
      total_dist += drone.total_distance
    return total_dist

  def get_total_collision_distance(self):
    total_dist = 0
    for collision in self.collisions:
      total_dist += collision.collision_distance
    return total_dist

  def get_risk_of_collision(self):
    total_dist = self.get_total_flights_distance()
    collision_distance = self.get_total_collision_distance()
    if total_dist == 0:
      return 0
    risk_of_collision = collision_distance / total_dist 
    return risk_of_collision

  def get_total_flights_duration(self):
    total_duration = 0
    for drone in self.drones:
      total_duration += drone.total_duration
    return total_duration
  
  def get_number_of_flights(self):
    no_of_flights = 0
    for drone in self.drones:
      no_of_flights += len(drone.flights)
    return no_of_flights

  def get_average_collisions_per_flight(self):
    num_flights = self.get_number_of_flights()
    if num_flights == 0:
      return 0
    return self.get_number_of_collisions() / num_flights


    