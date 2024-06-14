
import math
import re

from matplotlib import animation, pyplot as plt
from .Drone import Drone
from .Flight import Flight
from .Collision_Strategy import Collision_Strategy
import numpy as np
from .Swarm_Constants import *
import matplotlib.image as mpimg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from matplotlib.offsetbox import BboxImage, Bbox

class PF_Drone(Drone):
    def __init__(self, drone_id, drone, resolution_factor,  min_grid_offset=MIN_GRID_OFFSET, max_grid_offset=MAX_GRID_OFFSET):
        self.original_drone = drone
        super().__init__( drone.plan)
        self.drone_id = drone_id
        self.resolution_factor = resolution_factor
        self.min_grid_offset = min_grid_offset
        self.max_grid_offset = max_grid_offset
        self.grid_width = self.max_grid_offset - self.min_grid_offset
        self.cell_size = 1 / self.resolution_factor
        self.max_timesteps_per_flight = 50 // TIME_STEP
        self.current_timestep_for_flight = 0
        self.position = np.array(self.drone_to_grid(self.plan[0][0])) 
        self.positions = []
        self.positions.append(self.position.tolist())
        self.goals_reached = 0 # The first goal is reached by default
        self.current_wait_time = self.plan[0][1]
        self.timesteps = []
        if not self.original_drone.flights: # Some drones dont move
            self.finished = True
            self.at_last_goal = True
            self.flights_duration = self.plan[0][1]
            self.goal = np.array(self.plan[0][0])
            self.direction = np.array([0, 0])


        else:
            self.at_last_goal = False
            self.goals = [self.drone_to_grid(flight.flight_path[-1]) for flight in self.flights]
            self.goal = np.array(self.goals[0])
            self.direction = np.subtract(self.position, self.goals[0])
            self.finished = False

    # Map coordinates from drone space to grid space
    def drone_to_grid(self, drone_position):
       
        grid_position = [((coord - self.min_grid_offset) * self.resolution_factor)  for coord in drone_position]

        return grid_position
    
    # Map coordinates from grid space to drone space
    def grid_to_drone(self, grid_position):
        drone_position = [((coord) / self.resolution_factor) + self.min_grid_offset for coord in grid_position]
        #round positions 3dp
        drone_position = [round(coord, 3) for coord in drone_position]
        return drone_position
    
    def grid_distance(self, drone_distance):
        # Scale the input grid distance according to the resolution factor
        return drone_distance * self.resolution_factor
    def convert_to_flights(self):
        # Convert the positions of the drones to flights
        timesteps = self.timesteps
        new_plan = []
        drone = self
        flight_path = drone.positions
        if len(flight_path) == 1:
            return
        duplicate_positions = 0
        total_wait_time = 0
        pos = list(self.grid_to_drone(flight_path[0]))
        for i in range(len(flight_path) - 1):

            timestep = timesteps[i]
            if i == 0:
                duplicate_positions = 1
                total_wait_time = timestep

            destination = list(self.grid_to_drone(flight_path[i + 1]))
            

            if pos == destination:
                duplicate_positions += 1
                total_wait_time += timestep
                continue
        
           
         
            new_plan.append( [pos, (total_wait_time)])
            pos = destination
            duplicate_positions = 0
            total_wait_time = 0

        new_plan.append( [destination, drone.plan[-1][1]])

        drone.original_drone.plan = new_plan
        drone.plan = new_plan

        drone.original_drone.augment_plan(0, 0)
        drone.augment_plan(0, 0)




class Potential_Fields_Collision_Avoidance(Collision_Strategy):
    """
    A class that implements the potential fields collision avoidance strategy for drones.

    Parameters:
    - resolution_factor (int): The resolution factor for the grid size.
    - grid_size (int): The size of the grid.
    - visualise (bool): Flag indicating whether to visualize the potential fields.

    Methods:
    - detect_potential_collisions(drones): Detects potential collisions between drones.
    - potential_fields(drones): Calculates and adjusts the potential fields for drones.
    - calculate_potential_field(drone, drones): Calculates the potential field for a drone.
    - calculate_repulsion_from_drone(drone): Calculates the repulsion vectors from other drones.
    - calculate_attraction_to_goal(drone): Calculates the attraction vectors towards the goal.
    - animate_vector_fields(vector_fields, drone): Animates the vector fields for visualization.
    - adjust_drone_path(drones, drone, potential_field): Adjusts the drone's path based on the potential field.
    """

    def __init__(self, visualise=False):
        self.resolution_factor = self.calculate_resolution_factor(MINIMUM_DISTANCE, DISTANCE_STEP)
        self.timesteps = []
    
      
        self.visualise = visualise

    def detect_potential_collisions(self, drones):
        """
        Detects potential collisions between drones.

        Parameters:
        - drones (list): A list of drones.

        Returns:
        - bool: True if a collision is detected, False otherwise.
        """
        collision = False
        drone_positions = self.discretise_flight_paths(drones)
        for i in range(len(drones)):
            collision_flag = self.does_drone_collide(i, drone_positions)
            if collision_flag:
                collision = True
                break

        if collision:
            self.potential_fields(drones)

        return False  # Always return false as more iterations of potential fields are not going to help
    def smart_round(self, number):
        return round(number, 2)
    
    def calculate_resolution_factor(self, min_distance, step_distance):
        # Choose the minimum of min_distance and step_distance
        # min_val = min(min_distance, step_distance)
        min_val = min_distance
        min_val = max(min_val, 0.3) # prevent division by 0
        min_val /= 4  # Divide by 4 so the resolution gives chance to move away from drones
        # Calculate the resolution factor as the reciprocal of min_val
        resolution_factor = 1 / min_val

         #round up to nearst 1
        resolution_factor = self.smart_round(resolution_factor)

        return resolution_factor
    
    def calculate_grid_size(self, drones):
        """
        Calculates the grid size based on the drones' positions.

        Parameters:
        - drones (list): A list of drones.
        """
        # Initialize min and max values
        min_x = float('inf')
        min_y = float('inf')
        max_x = float('-inf')
        max_y = float('-inf')

        # Loop through each drone's plan
        for drone in drones:
            for position in drone.plan:
                x, y = position[0]
                # Update min and max values
                min_x = min(min_x, x)
                min_y = min(min_y, y)
                max_x = max(max_x, x)
                max_y = max(max_y, y)

        self.min_grid_offset = min(min_x, min_y) - (1/ self.resolution_factor) 
        self.max_grid_offset = max(max_x, max_y) + (1/ self.resolution_factor) 
        

        # grid size based on min and max grid offset
        self.grid_size = int((self.max_grid_offset - self.min_grid_offset) * self.resolution_factor)
        self.grid_size_original = self.grid_size


    def calculate_variable_timestep(self):
        # See if any drones have a current_wait_time that is less than the timestep
        smallest_timestep = TIME_STEP
        for drone in self.drones:
            if 0 < drone.current_wait_time < smallest_timestep:
                smallest_timestep = drone.current_wait_time
        return smallest_timestep
       

    def potential_fields(self, drones):
        """
        Calculates and adjusts the potential fields for drones.

        Parameters:
        - drones (list): A list of drones.
        """
        self.calculate_grid_size(drones)

        # Initialise PF_Drone objects
        drones = [PF_Drone(i, drone, self.resolution_factor, self.min_grid_offset, self.max_grid_offset) for i, drone in enumerate(drones)]
        self.number_of_drones = len(drones)
        self.drones = drones
        drones_not_done = len(drones)

        self.time_step = 0
        pfs_per_drone = [[] for _ in drones]
        count = 0
        drones_not_done = len(drones)
        self.calculate_repulsion_fields(drones)

        while drones_not_done > 0 and count < 600:
            count += 1
            if TIME_STEP < 1:
                self.current_timestep = self.calculate_variable_timestep()
            else:
                self.current_timestep = TIME_STEP
            for i, drone in enumerate(drones):

                if not drone.finished:
                    if drone.current_wait_time > 0:
                        if drone.current_wait_time < self.current_timestep:
                            drone.current_wait_time = 0
                        else:
                            drone.current_wait_time -= self.current_timestep

                    if drone.current_wait_time <= 0 and drone.at_last_goal:
                        drone.finished = True
                        drones_not_done -= 1
                        continue
                    
                    potential_field = self.calculate_potential_field(drone, drones)

                    if self.visualise:
                        # Add potential field to list for visualisation
                        pfs_per_drone[i].append(potential_field)

                    self.adjust_drone_path(drones, drone, potential_field, self.current_timestep)
                    drone.timesteps.append(self.current_timestep)
                    drone.current_PFG = self.calculate_repulsion_from_drone(drone)

            self.time_step += self.current_timestep
            self.time_step = round(self.time_step, 2)
        for i, drone in enumerate(drones):
            if self.visualise:
                self.animate_vector_fields(pfs_per_drone[i], drone, drones)

            drone.convert_to_flights()
    
    def calculate_repulsion_fields(self, drones):
        """
        Set the drone.current_PFG to the PFG that repels other drones from the drone

        Args:
            drones: The drones in the mission
        """

        for drone in drones:
            if not drone.finished:

                drone.current_PFG = self.calculate_repulsion_from_drone(drone)


    def calculate_potential_field(self, drone, drones):
        """
        Calculates the potential field for a drone.

        Parameters:
        - drone (PF_Drone): The drone for which to calculate the potential field.
        - drones (list): A list of drones.

        Returns:
        - np.ndarray: The calculated potential field.
        """
        # Initialize the potential field to zero
        potential_field = np.zeros((2, self.grid_size, self.grid_size))

        # Add the potential of each other drone
        for other_drone in drones:
            if other_drone != drone:
                if other_drone.finished == False:
                    # Calculate the potential of the other drone and add it to the potential field
                    obstacle_vectors = other_drone.current_PFG
                    potential_field += obstacle_vectors

        goal_potential = self.calculate_attraction_to_goal(drone)
        potential_field += goal_potential

        return potential_field

    def calculate_repulsion_from_drone(self, drone):
        """
        Calculates the repulsion vectors from other drones.

        Parameters:
        - drone (PF_Drone): The drone for which to calculate the repulsion vectors.

        Returns:
        - np.ndarray: The calculated repulsion vectors.
        """
        x = np.linspace(0, self.grid_size, self.grid_size)
        y = np.linspace(0, self.grid_size, self.grid_size)
        X, Y = np.meshgrid(x, y)
        drone_grid_position = drone.position.astype(int)
        # Calculate the vectors pointing away from the drone
        vectors = [X - drone_grid_position[0], Y - drone_grid_position[1]]
        distances = np.sqrt(vectors[0] ** 2 + vectors[1] ** 2)
        # Create a mask for distances within the effect distance
        minimum = 0 + 1e-9
        drone_priority = len(self.drones) - drone.drone_id
        maximum = drone.grid_distance((MINIMUM_DISTANCE * 2))# + (math.log(drone_priority, 20)))  # 3 times the minimum distance to give the drone change to move away
       
        mask = (distances >= minimum) & (distances <= maximum)

        # Apply the mask to the vectors
        vectors[0] = vectors[0] * mask
        vectors[1] = vectors[1] * mask
        
        # # Normalize the vectors
        magnitudes = np.sqrt(vectors[0] ** 2 + vectors[1] ** 2)
        magnitudes[magnitudes == 0] = 1  # Avoid division by zero
        
        # # Scale the vectors to the desired magnitude
        desired_repulsion = 2.5 ##+ (math.log(drone_priority, 20))

        vectors[0] = vectors[0] * desired_repulsion * desired_repulsion /  distances  / magnitudes
        vectors[1] = vectors[1] * desired_repulsion  * desired_repulsion  / distances / magnitudes
        
        # Clip the vectors (this mainly clips vectors right at drone position)
        vectors[0] = np.clip(vectors[0], -desired_repulsion, desired_repulsion)
        vectors[1] = np.clip(vectors[1], -desired_repulsion, desired_repulsion)
        
       
        return vectors

    def calculate_attraction_to_goal(self, drone):
        """
        Calculates the attraction vectors towards the goal.

        Parameters:
        - drone (PF_Drone): The drone for which to calculate the attraction vectors.

        Returns:
        - np.ndarray: The calculated attraction vectors.
        """
        x = np.linspace(0, self.grid_size, self.grid_size)
        y = np.linspace(0, self.grid_size, self.grid_size)
        X, Y = np.meshgrid(x, y)
        goal_grid_position = drone.goal.astype(int)
        # Calculate the vectors pointing towards the goal
        vectors = [goal_grid_position[0] - X, goal_grid_position[1] - Y]

        # Normalize the vectors to get unit vectors
        distances = np.sqrt(vectors[0] ** 2 + vectors[1] ** 2)
        unit_vectors = [vectors[0] / distances, vectors[1] / distances]


        return unit_vectors
    
    def visualize_vector_field(self, vector_field, title):
        """
        Visualizes a 2D vector field.

        Parameters:
        - vector_field (np.ndarray): The 2D vector field to visualize.
        """
        # Create a grid of coordinates
        y, x = np.mgrid[0:vector_field[0].shape[0], 0:vector_field[0].shape[1]]

        # Create a quiver plot
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.quiver(x, y, vector_field[0], vector_field[1])

        # Add a title
        ax.set_title(title)

        # Show the plot
        plt.show()
    
    def animate_vector_fields(self, vector_fields, drone, drones):
        """
        Animates the vector fields for visualization. Adds a drone image where the drone is
        and a target image where the current goal is

        Parameters:
        - vector_fields (list): A list of vector fields.
        - drone (PF_Drone): The drone associated with the vector fields.
        - drones (list): A list of other drones.
        """
        if len(drone.positions) == 1:
            return
        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(1, 1, 1)
        grid_size = vector_fields[0][0].shape[0] // 2
        y, x = np.mgrid[0:vector_fields[0][0].shape[0], 0:vector_fields[0][0].shape[1]]

        quiver = ax.quiver(x, y, vector_fields[0][0], vector_fields[0][1])  

        # Load the drone images
        drone_image = mpimg.imread('drone.png')
        other_drone_image = mpimg.imread('other_drone.png')
        target_image = mpimg.imread('target.png')
        
        goal_index = 0
        def update(i):
            nonlocal goal_index
            ax.clear()
            ax.quiver(x, y, vector_fields[i][0], vector_fields[i][1]) 
    
            # Create an OffsetImage for the main drone
            imagebox = OffsetImage(drone_image, zoom=0.07)
            adjusted_pos = drone.positions[i]
            ab = AnnotationBbox(imagebox, adjusted_pos, bboxprops=dict(alpha=0.0))
            ax.add_artist(ab)

            target_box = OffsetImage(target_image, zoom=0.14, alpha=0.48)
            target_ab = AnnotationBbox(target_box, drone.goals[goal_index], bboxprops=dict(alpha=0.0))
            ax.add_artist(target_ab)

            if np.array_equal(drone.positions[i], drone.goals[goal_index]) and goal_index < len(drone.goals) - 1:
                goal_index += 1
        
            # Create an OffsetImage for each other drone
            for other_drone in drones:
                if other_drone != drone:
                    if i >= len(other_drone.positions)-1:
                        other_adjusted_pos = other_drone.positions[-1]
                    else:
                        other_adjusted_pos = other_drone.positions[i]

                    
        
            ax.set_title('Vector Field for drone', pad=-40)
            
          

        ani = animation.FuncAnimation(fig, update, frames=len(vector_fields), repeat=False)
    
        plt.show()
    
    
    def adjust_drone_path(self, drones, drone, potential_field, time_step):
        """
        Adjusts the drone's path based on the potential field.

        Parameters:
        - drones (list): A list of drones.
        - drone (PF_Drone): The drone to adjust the path for.
        - potential_field (np.ndarray): The potential field for the drone.
        """
        # If drone shouldnt move, add its current position to its positions list for timestep consistency
        if drone.finished :
            return
        if drone.current_wait_time > 0:
            drone.positions.append(drone.position.tolist())

            return
        

        distance_step = time_step * SPEED
       
       
        grid_drone_position = (drone.position).astype(int)  # This can lose information, so we can increase resolution_factor if needed
        vector_at_drone = potential_field[:, grid_drone_position[1], grid_drone_position[0]]

        direction = vector_at_drone
        random_vector = np.random.normal(scale=0.05 , size=direction.shape)  # Add some noise to the direction to reduce local minima
        direction += random_vector

        # Check if goal is already occupied by other drone
        for other_drone in drones:
            if drone != other_drone:
                if np.array_equal(other_drone.position, drone.goal):
                    if other_drone.finished:
                        drone.finished = True
                        return

        distance_to_goal = np.linalg.norm(drone.position - drone.goal)

        if  distance_to_goal <= drone.grid_distance(distance_step):
            old_position = drone.position

            drone.positions.append(drone.goal.tolist())
            drone.position = (drone.goal)

            drone.goals_reached += 1

            if drone.goals_reached == len(drone.flights):
                drone.at_last_goal = True
                drone.current_wait_time = drone.plan[drone.goals_reached][1] + time_step

                return

            # Set the goal to the next flight's starting position
            drone.goal = np.array(drone.goals[drone.goals_reached])
            drone.current_wait_time = drone.plan[drone.goals_reached][1] + time_step
            drone.direction = np.subtract(drone.goal, drone.position)

        else:
            old_direction = drone.direction
            if np.linalg.norm(direction) != 0:
                new_direction = direction / np.linalg.norm(direction)  # normalised direction
                drone.direction = new_direction
         
            else:
                drone.direction = direction  # if direction vector is 0, keep it as 0

            # Move the drone a fixed distance in that direction if not out of bounds
            potential_pos = drone.position + drone.direction * (distance_step * self.resolution_factor)
            # Round potential position to nearest grid position
            potential_pos = np.round(potential_pos, 2).astype(int)
            # Check if the drone is moving out of bounds, stay if so

            if potential_pos[0] < 0 or potential_pos[0] > self.grid_size - 1 or potential_pos[1] < 0 or potential_pos[1] > self.grid_size - 1:
                drone.positions.append(drone.position.tolist())
                drone.direction = old_direction
            #if magnitude of direction is small and drone is close to goal, this means there is likely another drone at the goal, so wait
            elif np.linalg.norm(direction) < 0.2 and distance_to_goal <= drone.grid_distance((MINIMUM_DISTANCE * 2) + (math.log(len(self.drones), 20))):
                drone.positions.append(drone.position.tolist())


            else:
                drone.position = drone.position + drone.direction * (distance_step * self.resolution_factor)
                drone.positions.append(drone.position.tolist())
        

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

    # Check for collisions with a drone
    # Returns values (in this order): Boolean for collision, index i of other drone in collision, indexes of where the collision occurs in drone_positions
    def does_drone_collide(self, drone_index, drone_positions):

        # Check for collisions between drones with drone at drone_index
        for i in range(len(drone_positions)):
            if i == drone_index:
                continue
            collision_indexes = []
            for k in range(min(len(drone_positions[i]), len(drone_positions[drone_index]))):
                if math.dist(drone_positions[i][k], drone_positions[drone_index][k]) < MINIMUM_DISTANCE:
                    collision_indexes.append(k)
            if len(collision_indexes) > 0:
                return True, i, collision_indexes #return index of drone and position k where collision occurs
        return False, None, -1