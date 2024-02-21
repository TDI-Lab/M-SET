import math
from .Collision_Strategy import Collision_Strategy
from .Swarm_Constants import MINIMUM_DISTANCE
from .Swarm_Constants import SUBJECT_DRONE
from .Swarm_Constants import COMPARED_DRONE

class Basic_Collision_Avoidance(Collision_Strategy):
# A basic collision avoidance strategy class.
  def __init__(self, only_collision_detection = False):
    # Initialise the Basic Collection Detction/Avoidance strategy.
    self.only_collision_detection = only_collision_detection
    self.number_of_collisions = 0

  def detect_potential_collisions(self, drones):
    # Detect potential collisions by iterating through the drones and their flights.
    index = 0
    for drone in drones:
        index += 1
        flight_index = 0
        for flight in drone.flights:
            collision_flag, drone_to_augment = self.check_collision(flight, drones[index:])
            if not self.only_collision_detection:
                if drone_to_augment == SUBJECT_DRONE:
                    drone.augment_plan(flight_index)
                if collision_flag:
                    return True
            flight_index += 1
        
    return False


  def check_collision(self, subject_flight, drones):
    # Check for collision between a drone fligt and all other drone flights.
    for drone in drones:
        flight_index = 0
        for flight in drone.flights:
            if (self.same_air_time(subject_flight, flight)):
                collision_flag, drone_to_augment = self.compare_coordintes(subject_flight, flight)
                if not self.only_collision_detection:
                    if drone_to_augment == COMPARED_DRONE:
                        drone.augment_plan(flight_index)
                    if collision_flag:
                        return True, drone_to_augment
                flight_index += 1
    return False, None
                    

  def same_air_time(self, subject_flight, flight):
    # Check if two flights are airborne at the same time.
    if (subject_flight.start_time >= flight.start_time and subject_flight.start_time < flight.finish_time):
        return True
    elif (subject_flight.start_time < flight.start_time and subject_flight.finish_time >= flight.start_time):
        return True
    return False


  def compare_coordintes(self, subject_flight, flight):
    # Check if two flights that are airborne at the same time get too close at any point of time.
    comparison_from = max(subject_flight.start_time, flight.start_time)
    comparison_until = min(subject_flight.finish_time, flight.finish_time)

    for i in range(comparison_from, comparison_until):
        subjet_flight_index = i - subject_flight.start_time
        flight_index = i - flight.start_time
        distance_between_drones = math.dist(subject_flight.flight_path[subjet_flight_index], flight.flight_path[flight_index])
        if (distance_between_drones <= MINIMUM_DISTANCE):
            self.number_of_collisions += 1
            if (subject_flight.start_time <= flight.start_time):
                return True, COMPARED_DRONE
            else: 
                return True, SUBJECT_DRONE
    
    return False, None





            

