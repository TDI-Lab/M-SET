import math
import matplotlib.pyplot as plt
from .Drone import Drone
from .Collision_Strategy import Collision_Strategy
from .Basic_Collision_Avoidance import Basic_Collision_Avoidance
from .Offline_Collision_Stats import Offline_Collision_Stats
from .Swarm_Constants import COLLISION_AVOIDANCE_LIMIT, GRID_SIZE, MAX_GRID_OFFSET, MIN_GRID_OFFSET, SPEED, TIME_STEP, INTERPOLATION_FACTOR, FRAMES_PER_TIMESTEP
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation, FFMpegWriter
from random import randint

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

  def drones_to_plan(self):
    # Get the plans of the drones.
    self.plans = [drone.plan for drone in self.drones]
  def detect_potential_collisions(self):
    # Detect and fix potential collisions between drones.
    for i in range(COLLISION_AVOIDANCE_LIMIT):
      if (not self.collision_strategy.detect_potential_collisions(self.drones)):
        self.drones_to_plan()

        return
  
  def get_offline_collision_stats(self):
    # Get collision data.
    # TODO: Add more stats.
    self.initialise_drones()
    offline_collision_stats = Offline_Collision_Stats(self.drones)

    collision_data = {
      'number_of_collisions': offline_collision_stats.get_number_of_collisions(),
      'number_of_cross_collisions': offline_collision_stats.get_number_of_cross_collisions(),
      'number_of_parallel_collisions': offline_collision_stats.get_number_of_parallel_collisions(),
      'number_of_dest_occupied_collisions': offline_collision_stats.get_number_of_dest_occupied_collisions(),
      'total_flights_distance': round(offline_collision_stats.get_total_flights_distance(), 3),
      'total_collision_distance': round(offline_collision_stats.get_total_collision_distance(), 3),
      'risk_of_collision': round(offline_collision_stats.get_risk_of_collision(), 3),
      'total_duration_of_flights': offline_collision_stats.get_total_flights_duration(),
      'number_of_flights': offline_collision_stats.get_number_of_flights(),
      'average_collisions_per_flight': round(offline_collision_stats.get_average_collisions_per_flight(), 3),
      'total_hover_duration': offline_collision_stats.get_total_hover_time()
    }

    print('\nCollision Data: \n')
    for key, value in collision_data.items():
      print(f'{key:50}= {value}')
    
    return collision_data

 
  def print_itinerary(self):
    print("Drone plans: ", *self.plans,sep='\n')
    print("\nDrones: \n\n")
    drone_index = 0
    for drone in self.drones:
      print("\nDrone ", drone_index, ": \n")
      drone.print_itinerary()
      drone_index += 1
      print("**********************************")


  def discretise_flight_paths(self, drones, from_time=None, to_time=None):
    drone_positions = []
    if to_time is None:
        to_time = max(drone.flights[-1].finish_time for drone in drones if drone.flights) if any(drone.flights for drone in drones) else 0
    if from_time is None:
        from_time = min(drone.flights[0].start_time for drone in drones if drone.flights) if any(drone.flights for drone in drones) else 0

    num_positions = int(math.ceil((to_time - from_time) / TIME_STEP)) + 1

    drones_with_no_flights = [i for i, drone in enumerate(drones) if not drone.flights]

    for i, drone in enumerate(drones):
        if i in drones_with_no_flights:
            drone_positions.append([drone.plan[0][0]] * num_positions)
        else:
            positions_for_drone = [None] * num_positions

            flight_i = 0
            for flight in drone.flights:
                if flight.finish_time < from_time or flight.start_time > to_time:
                    continue  # Skip flights outside the time range

                start_index = max(0, int((flight.start_time - from_time) / TIME_STEP))
                end_index = min(num_positions, start_index + len(flight.flight_path))

                for i in range(start_index, end_index):
                    if positions_for_drone[i] is None:  # Avoid duplicate positions
                        positions_for_drone[i] = flight.flight_path[i - start_index]

            # Fill in gaps with the last known position
            last_known_position = None
            for i in range(num_positions):
                if positions_for_drone[i] is not None:
                    last_known_position = positions_for_drone[i]
                elif last_known_position is not None:
                    positions_for_drone[i] = last_known_position

            # If the drone's first flight starts after from_time, prepend positions with the first known position
            if flight_i == 0 and flight.start_time > from_time:
                first_known_position = drone.flights[0].flight_path[0]
                for i in range(num_positions):
                    if positions_for_drone[i] is None:
                        positions_for_drone[i] = first_known_position
                    else:
                        break
            flight_i += 1
            # If the drone's last flight finishes before to_time, extend positions with the last known position
            if drone.flights[-1].finish_time < to_time:
                positions_for_drone += [last_known_position] * (num_positions - len(positions_for_drone))

            drone_positions.append(positions_for_drone)
    for drone in drone_positions:
        for pos in drone:
            if pos is None:
                print("Error: None position detected. Flights not proerly discretised. Exiting...")
                return
    return drone_positions


  def visualise_swarm(self, title="", save=False, filepath=None):
    discreet_positions = self.discretise_flight_paths(self.drones)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    scatters = []
    for i in range(len(discreet_positions)):
        scatter = ax.scatter([], [], [])
        scatters.append(scatter)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    ax.set_xlim([MIN_GRID_OFFSET, MAX_GRID_OFFSET])  
    ax.set_ylim([MIN_GRID_OFFSET, MAX_GRID_OFFSET])  
    ax.set_zlim([MIN_GRID_OFFSET, MAX_GRID_OFFSET])  

    ax.set_title(title)

    # Create text box for time
    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes)

    # Update function for the animation
    def update(frame, scatters, discreet_positions):
      t = frame / FRAMES_PER_TIMESTEP
      num = int(math.floor(t))
      t = t - num
      for i, drone_positions in enumerate(discreet_positions):
        if num < len(drone_positions) - 1:
          pos1 = drone_positions[num]
          pos2 = drone_positions[num + 1]
          x = (1 - t) * pos1[0] + t * pos2[0]
          y = (1 - t) * pos1[1] + t * pos2[1]
          scatters[i]._offsets3d = ([[x], [y], [3]])
      # Update the time text
      time_text.set_text(f'Time: {frame * (FRAMES_PER_TIMESTEP/TIME_STEP)/100 :.2f} s')

      
    total_frames = int(len(discreet_positions[0])  * FRAMES_PER_TIMESTEP )

    ani = FuncAnimation(fig, update, fargs=(scatters, discreet_positions), frames=total_frames, interval=(FRAMES_PER_TIMESTEP/TIME_STEP)/100, blit=False, repeat=False)
     
  
    plt.show()

  @staticmethod
  def determine_priority(plans):
    # Generate priority map with randomly assigned priority to cells.
    grid = []
    for drone in plans:
        for plan in drone:
            coordinates = plan[0]
            if coordinates not in grid:
                grid.append(coordinates)

    priority = set(range(1, len(grid) + 1))
    used_priorities = []
    def get_random_priority():
        reduced_list = list(priority - set(used_priorities))
        i = randint(0, len(reduced_list) - 1)
        used_priorities.append(reduced_list[i])
        return reduced_list[i]
    
    priority_map = {} 
    for i in range(0, len(grid)):
        next_priority =  get_random_priority()
        priority_map[next_priority] = grid[i]
        
    return priority_map