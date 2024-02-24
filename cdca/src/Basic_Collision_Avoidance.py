import math
from .Collision_Strategy import Collision_Strategy
from .Swarm_Constants import *

class Collision:
  def __init__(self, collision_point_A, collision_point_B, start_time_A, start_time_B, collision_type, collision_time):
    self.drones = {
        'drone_A': {'collision_point' : collision_point_A, 'flight_start_time' : start_time_A}, 
        'drone_B': {'collision_point' : collision_point_B, 'flight_start_time' : start_time_B}
    }
    self.collision_type = collision_type
    self.collision_time = collision_time

  def set_drone_A(self, drone, flight_index):
    self.drones['drone_A']['drone'] = drone
    self.drones['drone_A']['flight_index'] = flight_index

  def set_drone_B(self, drone, flight_index):
    self.drones['drone_B']['drone'] = drone
    self.drones['drone_B']['flight_index'] = flight_index


class Basic_Collision_Avoidance(Collision_Strategy):
# A basic collision avoidance strategy class.
  def __init__(self, only_collision_detection = False):
    # Initialise the Basic Collection Detction/Avoidance strategy.
    self.only_collision_detection = only_collision_detection
    self.number_of_collisions = 0
    self.collisions = []

  def detect_potential_collisions(self, drones):
    # Detect potential collisions by iterating through the drones and their flights.
    index = 0
    for drone in drones:
        index += 1
        flight_index = 0
        for flight in drone.flights:
            collision_flag, drone_to_augment, collision_type = self.check_collision(flight, drones[index:])
            if not self.only_collision_detection:
                if drone_to_augment == SUBJECT_DRONE:
                    self.augment_plan(drone, flight_index, collision_type)
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
                collision_flag, drone_to_augment, collision_type = self.compare_coordintes(subject_flight, flight)
                if not self.only_collision_detection:
                    if drone_to_augment == COMPARED_DRONE:
                        self.augment_plan(drone, flight_index, collision_type)
                    if collision_flag:
                        return True, drone_to_augment, collision_type
                flight_index += 1
    return False, None, None
                    

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

        point_A = subject_flight.flight_path[subjet_flight_index]
        point_B = flight.flight_path[flight_index]

        distance_between_drones = math.dist(point_A, point_B)
        if (distance_between_drones <= MINIMUM_DISTANCE):
            collision_type = self.identify_collision_type(subject_flight, flight)
            if (collision_type == 1):
                print("****************************CROSS")
            else:
                print("****************************PARALLEL")
            new_collision = Collision(point_A, point_B, subject_flight, flight, collision_type, i)
            self.collisions.append(new_collision)
            self.number_of_collisions += 1
            if (subject_flight.start_time <= flight.start_time):
                return True, COMPARED_DRONE, collision_type
            else: 
                return True, SUBJECT_DRONE, collision_type
    
    return False, None, None 

  def identify_collision_type(self, subject_flight, flight):
    number_of_close_points = 0
    for i in range(len(subject_flight.flight_path)):
        for j in range(len(flight.flight_path)):
            distance_between_points = math.dist(subject_flight.flight_path[i], flight.flight_path[j])
            # print("distance_between_points: ", distance_between_points)
            if (distance_between_points <= MINIMUM_DISTANCE):
                number_of_close_points += 1
    if (number_of_close_points <  MIN_POINTS_PARALLEL_COLLISION):
        return CROSS_COLLISION
    else:
        return PARALLEL_COLLISION
        
  def augment_plan(self, drone, flight_index, collision_type):
    delay = 0
    if (collision_type == CROSS_COLLISION):
        delay = TIME_DELAY
    elif (collision_type == PARALLEL_COLLISION):
        drone.plan.insert(flight_index + 1, [[2, 8], 0])
        delay = 0
    else:
        print("augmentation for this collision is not implemented yet.")
        return 
    drone.augment_plan(flight_index, delay)







            

