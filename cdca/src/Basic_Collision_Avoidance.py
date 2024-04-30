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

  def get_drone_to_augment(self, priority_map):
      if self.collision_type == DEST_OCCUPIED_COLLISION:
        if (self.drones['drone_A']['flight_start_time'] <= self.drones['drone_B']['flight_start_time']):
            return self.drones['drone_A'], self.drones['drone_B']
        else:
            return self.drones['drone_B'], self.drones['drone_A']
      elif priority_map is None:
        if (self.drones['drone_A']['flight_start_time'] <= self.drones['drone_B']['flight_start_time']):
            return self.drones['drone_B'], self.drones['drone_A']
        else:
            return self.drones['drone_A'], self.drones['drone_B']
      else:
        drone_A_dest = self.drones['drone_A']['drone'].flights[self.drones['drone_A']['flight_index']].destination
        drone_B_dest = self.drones['drone_B']['drone'].flights[self.drones['drone_B']['flight_index']].destination
        priority_A = list(priority_map.keys())[list(priority_map.values()).index(drone_A_dest)]
        priority_B = list(priority_map.keys())[list(priority_map.values()).index(drone_B_dest)]
        
        if (priority_A < priority_B):
          return self.drones['drone_B'], self.drones['drone_A']
        else:
          return self.drones['drone_A'], self.drones['drone_B']


  def set_collision_distance(self, collision_points = None):
    if collision_points is not None:
      self.collision_distance = collision_points * DISTANCE_STEP * 2
    else:
      self.collision_distance = MINIMUM_DISTANCE
  

  def print_collision(self):
    print(self.drones)
    print(self.collision_type)
    print(self.collision_time)


class Basic_Collision_Avoidance(Collision_Strategy):
# A basic collision avoidance strategy class.
  def __init__(self, only_collision_detection = False, priority_map = None):
    # Initialise the Basic Collection Detction/Avoidance strategy.
    self.only_collision_detection = only_collision_detection
    self.collisions = []
    self.priority_map = priority_map

  def detect_potential_collisions(self, drones):
    # Detect potential collisions by iterating through the drones and their flights.
    drone_index = 0
    for drone in drones:
        drone_index += 1
        flight_index = 0
        for flight in drone.flights:
            collision_flag = self.check_collision(flight, drones, drone_index)
            if collision_flag:
                self.collisions[-1].set_drone_A(drone, flight_index)
                if not self.only_collision_detection:
                    self.augment_plan()
                    return True
            flight_index += 1
        
    return False


  def check_collision(self, subject_flight, drones, subject_drone_index):
    # Check for collision between a drone fligt and all other drone flights.
    drone_index = 0
    for drone in drones:
        drone_index += 1
        if drone_index == subject_drone_index:
            continue
        flight_index = 0
        prev_flight = None

        for flight in drone.flights:
            collision_flag = False
            if self.destination_occupied(subject_flight, flight, prev_flight):
                self.identify_collision_type(subject_flight, flight)
                new_collision = Collision(subject_flight.flight_path[-1], flight.flight_path[0], subject_flight.start_time, flight.start_time, DEST_OCCUPIED_COLLISION, subject_flight.finish_time)
                self.collisions.append(new_collision)
                self.collisions[-1].set_collision_distance()
                collision_flag = True

            elif (self.same_air_time(subject_flight, flight) and drone_index > subject_drone_index):
                collision_flag = self.compare_coordintes(subject_flight, flight)

            if collision_flag:
                self.collisions[-1].set_drone_B(drone, flight_index)
                if not self.only_collision_detection:
                    return True

            flight_index += 1
            prev_flight = flight

    return False

  def destination_occupied(self, subject_flight, flight, prev_flight):
    # Check if a drone is present at the destination
    if subject_flight.finish_time <= flight.start_time:
      if not prev_flight == None:
        if prev_flight.finish_time > subject_flight.finish_time:
          return False
      if subject_flight.flight_path[-1][0] == flight.flight_path[0][0] and subject_flight.flight_path[-1][1] == flight.flight_path[0][1]:
        return True
    return False
                    

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
    comparison_iterations = int((comparison_until - comparison_from) / TIME_STEP)
    
    comparison_range = []
    for i in range(comparison_iterations + 1):
        comparison_range.append(comparison_from + (i * TIME_STEP))
      
    collision_flag = False
    collision_points = 0
    for i in comparison_range:
        subjet_flight_index = int((i - subject_flight.start_time) / TIME_STEP)
        flight_index = int((i - flight.start_time) / TIME_STEP)

        point_A = subject_flight.flight_path[subjet_flight_index]
        point_B = flight.flight_path[flight_index]

        distance_between_drones = math.dist(point_A, point_B)
        if (distance_between_drones <= MINIMUM_DISTANCE):
            if not collision_flag:
                collision_type = self.identify_collision_type(subject_flight, flight)
                new_collision = Collision(point_A, point_B, subject_flight.start_time, flight.start_time, collision_type, i)
                self.collisions.append(new_collision)
                collision_points += 1
                collision_flag = True
            else:
                collision_points += 1

    if collision_flag:
        self.collisions[-1].set_collision_distance(collision_points)
        return True
    return False


  def identify_collision_type(self, subject_flight, flight):
    # Identify what kind of a collision will occur.
    number_of_close_points = 0
    for i in range(len(subject_flight.flight_path)):
        for j in range(len(flight.flight_path)):
            distance_between_points = math.dist(subject_flight.flight_path[i], flight.flight_path[j])
            if (distance_between_points <= MINIMUM_DISTANCE):
                number_of_close_points += 1
    if (number_of_close_points <  MIN_POINTS_PARALLEL_COLLISION):
        return CROSS_COLLISION
    else:
        return PARALLEL_COLLISION

        
  def augment_plan(self):
    # Augment drone plans based on collision type.
    collision = self.collisions[-1]
    drone_to_augment, other_drone = collision.get_drone_to_augment(self.priority_map)
    delay = 0

    if (collision.collision_type == CROSS_COLLISION):
        delay = TIME_DELAY

    elif (collision.collision_type == PARALLEL_COLLISION):
        new_point = self.get_orthogonal_point(drone_to_augment['collision_point'], drone_to_augment['drone'].plan[drone_to_augment['flight_index']][0])
        drone_to_augment['drone'].plan.insert(drone_to_augment['flight_index'] + 1, [new_point, 0])
        delay = 0
    
    elif (collision.collision_type == DEST_OCCUPIED_COLLISION):
        delay = + TIME_DELAY + other_drone['flight_start_time'] - drone_to_augment['drone'].flights[drone_to_augment['flight_index']].finish_time

    else:
        print("Augmentation for this type of collision has not bee implemented yet.")
        return 

    drone_to_augment['drone'].augment_plan(drone_to_augment['flight_index'], delay)


  def get_orthogonal_point(self, collision_point, start_point, orthogonal_dist=PARALLEL_CA_ORTH_DIST):
    # Get a point on the line perpendicular to the drones original flight. 
    # The perpendicular line passes through the collision point.
    y_difference = collision_point[1] - start_point[1]
    x_difference = start_point[0] - collision_point[0] 
    multiplier = orthogonal_dist / (math.sqrt((y_difference ** 2) + (x_difference ** 2)) + 1e-10)

    orthogonal_line = [y_difference * multiplier, x_difference * multiplier]

    orthogonal_point = [collision_point[0] - orthogonal_line[0], collision_point[1] - orthogonal_line[1]]


    return orthogonal_point








            

