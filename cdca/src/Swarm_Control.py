import math
import matplotlib.pyplot as plt
from .Drone import Drone
from .Collision_Strategy import Collision_Strategy
from .Basic_Collision_Avoidance import Basic_Collision_Avoidance
from .Offline_Collision_Stats import Offline_Collision_Stats
from .Swarm_Constants import COLLISION_AVOIDANCE_LIMIT, SPEED, TIME_STEP, INTERPOLATION_FACTOR, FRAMES_PER_TIMESTEP
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation

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
  
  def get_offline_collision_stats(self):
    # Get collision data.
    # TODO: Add more stats.
    offline_collision_stats = Offline_Collision_Stats(self.drones)

    collision_data = {
      'number_of_collisions': offline_collision_stats.get_number_of_collisions(),
      'number_of_cross_collisions': offline_collision_stats.get_number_of_cross_collisions(),
      'number_of_parallel_collisions': offline_collision_stats.get_number_of_parallel_collisions(),
      'number_of_dest_occupied_collisions': offline_collision_stats.get_number_of_dest_occupied_collisions()
    }

    print(collision_data)
    
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
            to_time = max(drone.flights[-1].finish_time for drone in drones)
        if from_time is None:
            from_time = min(drone.flights[0].start_time for drone in drones)

        num_positions = int(math.ceil((to_time - from_time) / TIME_STEP)) + 1
    

        for drone in drones:
            positions_for_drone = []

            check_flight_time_range = lambda flight, time_from, time_to: time_from <= flight.start_time <= time_to or time_from <= flight.finish_time <= time_to
            flights_in_time = [flight for flight in drone.flights if check_flight_time_range(flight, from_time, to_time)]
            if len(flights_in_time) == 0:
                #get last known position of drone
                flights_before_time = [flight for flight in drone.flights if flight.finish_time < from_time]
                pos = flights_before_time[-1].flight_path[-1]
                positions_for_drone.extend([pos] * num_positions)
                drone_positions.append(positions_for_drone)

                continue
            for i, flight in enumerate(flights_in_time): 
                if i > 0 and flights_in_time[i - 1].finish_time == flight.start_time:
                    positions_for_drone.pop(-1)
                if i == 0 and flight.start_time < from_time: # Cut off positions of first flight that is before from_time
                    if ((from_time - flight.start_time) *10) % (TIME_STEP*10) != 0: # Check if the time difference is not a multiple of TIME_STEP
                        prefix_flight_num = int(round(flight.finish_time - from_time) / TIME_STEP) + 2
                    else:
                        prefix_flight_num = int(round(flight.finish_time - from_time) / TIME_STEP) + 1
                   
                    extra = max(0, prefix_flight_num - len(flight.flight_path))

                    prefix_flight_num -= extra

                    positions_for_drone.extend(flight.flight_path[-prefix_flight_num:])
                    # positions_for_drone.extend([flight.flight_path[0]] * extra) # Add the first position of the flight to the drone's path using list multiplication
                elif i == 0 and flight.start_time > from_time: # Add positions to start if first flight is after from_time

                    prefix_flight_num = int((flight.start_time - from_time) / TIME_STEP) 
                    positions_for_drone.extend([flight.flight_path[0]] * prefix_flight_num)
                    positions_for_drone.extend(flight.flight_path)

                else:
                    positions_for_drone.extend(flight.flight_path)

                # Add positions for the time between this flight and the next flight
                if i < len(flights_in_time) - 1 and flights_in_time[i + 1].start_time > flight.finish_time:
                    if len(flights_in_time) == 1:
                        gap_flight_num = int((flights_in_time[i + 1].start_time - flight.finish_time) / TIME_STEP) - 2
                    else:
                        gap_flight_num = int((flights_in_time[i + 1].start_time - flight.finish_time) / TIME_STEP) - 1
                    positions_for_drone.extend([flight.flight_path[-1]] * gap_flight_num)

            if flights_in_time[-1].finish_time < to_time:
                suffix_flight_num = int(round((to_time - round(flights_in_time[-1].finish_time, 2)) / TIME_STEP) )
                positions_for_drone.extend([flights_in_time[-1].flight_path[-1]] * suffix_flight_num)

            assert len(positions_for_drone)   == num_positions
            drone_positions.append(positions_for_drone)

        return drone_positions

  
  def visualise_swarm(self):
    # Visualise the swarm.
    discreet_positions = self.discretise_flight_paths(self.drones)

    # Create a figure and a 3D subplot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Create a scatter plot for each drone
    scatters = []
    for i in range(len(discreet_positions)):
        scatter = ax.scatter([], [], [], label=f'Drone {i+1}', s=50)
        scatters.append(scatter)

    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Set axes limits
    ax.set_xlim([0, 10])  
    ax.set_ylim([0, 10])  
    ax.set_zlim([0, 10])  


    # Create a text box for the time
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

    # Create the animation
      
    total_frames = int(len(discreet_positions[0])  * FRAMES_PER_TIMESTEP )

    ani = FuncAnimation(fig, update, fargs=(scatters, discreet_positions), frames=total_frames, interval=(FRAMES_PER_TIMESTEP/TIME_STEP)/100, blit=False, repeat=False)

    plt.show()