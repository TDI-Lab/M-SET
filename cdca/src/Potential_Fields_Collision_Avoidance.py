
import math

from matplotlib import animation, pyplot as plt
from src.Drone import Drone
from src.Flight import Flight
from src.Collision_Strategy import Collision_Strategy
import numpy as np
from src.Swarm_Constants import *
from mpl_toolkits.mplot3d import Axes3D

class PF_Drone(Drone):
    def __init__(self, drone_id, drone, resolution_factor):
        self.original_drone = drone
        super().__init__( drone.plan)
        self.drone_id = drone_id

        self.resolution_factor = resolution_factor
        self.goals = [self.drone_to_grid(flight.flight_path[-1]) for flight in self.flights]
        self.position = np.array(self.drone_to_grid(self.plan[0][0])) 
        self.direction = np.subtract(self.position, self.goals[0])
        self.goal = np.array(self.goals[0])
        self.positions = []
        self.goals_reached = 0 # The first goal is reached by default
        self.current_wait_time = self.plan[0][1]
        self.finished = False

    def drone_to_grid(self, drone_position):
        return (drone_position[0] * self.resolution_factor), (drone_position[1] * self.resolution_factor)

    def grid_to_drone(self, grid_position):
        return (grid_position[0] / self.resolution_factor), (grid_position[1] / self.resolution_factor)

    def convert_to_flights(self):
        # Convert the positions of the drones to flights
        goal = 0
        drone = self
        flight_path = drone.positions
        index = 1
        delay = 0

        for i in range(len(flight_path) - 1):
            prev_destination = list(self.grid_to_drone([flight_path[i][0], flight_path[i][1]]))

            destination = list(self.grid_to_drone([flight_path[i + 1][0], flight_path[i + 1][1]]))

            if prev_destination == destination:
                # delay += TIME_STEP
                continue

            
            #if destination is .0:
            if destination[0] % 1 == 0:
                destination[0] = int(destination[0])
            if destination[1] % 1 == 0:
                destination[1] = int(destination[1])



            if float(destination[0]) == float(drone.plan[goal][0][0]) and float(destination[1]) == float(drone.plan[goal][0][1]):
                
                continue
            elif float(destination[0]) == float(drone.plan[goal+1][0][0]) and float(destination[1]) == float(drone.plan[goal+1][0][1]):
                goal += 1
                continue
            #if drone is within certain distance of goal, move to next goal
            elif math.dist(destination, drone.plan[goal-1][0]) < DISTANCE_STEP or math.dist(destination, drone.plan[goal][0]) < DISTANCE_STEP:
                # goal += 1
                continue                
            else:
                drone.original_drone.plan.insert(goal + index, [destination, delay])

                index += 1

        #if final position is more than DISTANCE_STEP away from goal, remove final plan as it is not reached
        if math.dist(destination, drone.plan[-1][0]) > DISTANCE_STEP:
            last_plan = drone.original_drone.plan.pop(-1)
            secondlast_plan = drone.original_drone.plan.pop(-1) #remove the last two plans for good measure (if it hasnt reached the last goal, there is probably a drone already there)
            print("Last position not reached: ", last_plan[0])
            print("Removing from flights")
            drone.original_drone.plan[-1][1] = secondlast_plan[1] + last_plan[1] + drone.original_drone.plan[-1][1]
        drone.original_drone.augment_plan(0, 0)



class Potential_Fields_Collision_Avoidance(Collision_Strategy):
    def __init__(self, resolution_factor=2, visualise=False):
        self.resolution_factor = resolution_factor
        self.grid_size = GRID_SIZE * self.resolution_factor
        self.visualise = visualise

    def detect_potential_collisions(self, drones):
        # Detect potential collisions by iterating through the drones and their flights.
        collision = False
        drone_positions = self.discretise_flight_paths(drones)
        for i in range(len(drones)):

            collision_flag = self.does_drone_collide( i, drone_positions)
            if collision_flag:
                collision = True
                break

        if collision:
            self.potential_fields(drones)

        return False #Always return false as more iterations of potential fields are not going to help
    
    def potential_fields(self, drones):
        # Initialise PF_Drone objects
        drones = [PF_Drone(i, drone, self.resolution_factor) for i, drone in enumerate(drones)]
        self.number_of_drones = len(drones)

        drones_not_done = len(drones)

        self.time_step = 0
        pfs_per_drone = [[] for _ in drones]
        count = 0
        while (drones_not_done) > 0 and count < 100:
            count += 1

            for i, drone in enumerate(drones):

               

                if drone.current_wait_time > 0:
                    drone.current_wait_time -= TIME_STEP

                    
                potential_field = self.calculate_potential_field(drone, drones)
                # magnitudes = np.sqrt(potential_field[0]**2 + potential_field[1]**2)
                # print("Magnitudes for potential field: ", magnitudes)
                # Add potential field to list for visualisation
                pfs_per_drone[i].append(potential_field)
                
                # Dont move drones that have reached all their goals, still calculate their potential fields for visualiations
                # if not drone.finished or drone.current_wait_time != 0:
                self.adjust_drone_path(drone, potential_field)
            
            self.time_step += TIME_STEP
            self.time_step = round(self.time_step, 2)
        for i, drone in enumerate(drones):
            if self.visualise:
                self.animate_vector_fields(pfs_per_drone[i], drone)
            drone.convert_to_flights()

    def calculate_potential_field(self, drone, drones):
        # Initialize the potential field to zero
        potential_field = np.zeros((2,self.grid_size, self.grid_size))

        # Add the potential of each other drone
        for other_drone in drones:
            if other_drone != drone:
                # Calculate the potential of the other drone and add it to the potential field
                obstacle_vectors = self.calculate_repulsion_from_drone( other_drone)
                # self.visualize_gradient(potential)
                # Apply the drone potential n times
                # for _ in range(1):
                potential_field += obstacle_vectors  # Negative attraction from other drones


        # Add the potential of the goal
        goal_potential = self.calculate_attraction_to_goal(drone)
        # self.visualize_gradient(goal_potential, gradient)
        potential_field += goal_potential  # Positive attraction to goal
        # self.visualize_gradient(potential_field)
        return potential_field
    

  

    def calculate_repulsion_from_drone(self, drone):
        x = np.linspace(0, self.grid_size, self.grid_size)
        y = np.linspace(0, self.grid_size, self.grid_size)
        X, Y = np.meshgrid(x,y)
        # Calculate the vectors pointing away from the drone
        vectors = [X - drone.position[0], Y - drone.position[1]]
        distances = np.sqrt(vectors[0]**2 + vectors[1]**2)
        # Create a mask for distances within the effect distance
        minimum = 0 + 1e-9
        maximum = MINIMUM_DISTANCE * self.resolution_factor * 10
        mask = (distances >= minimum) & (distances <= maximum)

        # Apply the mask to the vectors and divide by the square of the distances
        vectors[0] = vectors[0] * mask / (distances**2 + 1e-9)
        vectors[1] = vectors[1] * mask / (distances**2 + 1e-9)

        # Scale the vectors by a factor that is inversely proportional to the distance
        scale_factor = maximum / (distances + 1e-9)
        vectors[0] *= scale_factor
        vectors[1] *= scale_factor

        # Cap the magnitudes of the vectors based on drone index
        clip_range = np.clip(self.number_of_drones - drone.drone_id, 0.5, 3)
        vectors[0] = np.clip(vectors[0], -clip_range, clip_range)
        vectors[1] = np.clip(vectors[1], -clip_range, clip_range)

        return vectors

    def calculate_attraction_to_goal(self, drone):
        x = np.linspace(0, self.grid_size, self.grid_size)
        y = np.linspace(0, self.grid_size, self.grid_size)
        X, Y = np.meshgrid(x,y)

        # Calculate the vectors pointing towards the goal
        vectors = [drone.goal[0] - X, drone.goal[1] - Y]

        # Normalize the vectors to get unit vectors
        distances = np.sqrt(vectors[0]**2 + vectors[1]**2)
        unit_vectors = [vectors[0] / distances, vectors[1] / distances]
        
        # Set the vectors at positions a specified distance away from the goal to zero
        distance_threshold = (DISTANCE_STEP) * self.resolution_factor
        goal_position = drone.goal.astype(int)
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                distance_to_goal = np.linalg.norm(np.array([i, j]) - goal_position)
                if distance_to_goal <= distance_threshold:
                    unit_vectors[0][j, i] = 0
                    unit_vectors[1][j, i] = 0
        
        return unit_vectors


    

    def animate_vector_fields(self, vector_fields, drone):
        fig = plt.figure(figsize=(7,7))
        ax = fig.add_subplot(1, 1, 1)
        y, x = np.mgrid[0:vector_fields[0][0].shape[0], 0:vector_fields[0][0].shape[1]]

        quiver = ax.quiver(x, y, vector_fields[0][0], vector_fields[0][1])
        ax.scatter(*(drone.positions[0]), color='b')

        def update(i):
            ax.clear()
            ax.quiver(x, y, vector_fields[i][0], vector_fields[i][1])
            # if i > len(drone.positions) - 1:
            #     ax.scatter(*drone.positions[-1], color='b')
            # else:
            ax.scatter(*drone.positions[i], color='b')
            ax.set_title('Vector Field for drone')

        ani = animation.FuncAnimation(fig, update, frames=len(vector_fields), repeat=False)

        plt.tight_layout()
        plt.show()
    def adjust_drone_path(self, drone, potential_field):

        # If drone shouldnt move, add its current position to its positions list for timestep consistency
        if drone.finished or drone.current_wait_time != 0:
            drone.positions.append(drone.position.tolist())
            return


        grid_drone_position = (drone.position).astype(int) #This can lose information, so we can increase resolution_factor if needed
        vector_at_drone = potential_field[:, grid_drone_position[1], grid_drone_position[0]]

        direction = vector_at_drone
        random_vector = np.random.normal(scale=0.1, size=direction.shape) # Add some noise to the direction to reduce local minima
        direction += random_vector

    
        distance_to_goal = np.linalg.norm(drone.position - drone.goal)

        if distance_to_goal <= (DISTANCE_STEP * self.resolution_factor):
            drone.positions.append(drone.goal.tolist())
            drone.position = (drone.goal)

            drone.goals_reached += 1

            # If the drone is close enough to the goal, increment the number of goals reached
            # If the drone has reached all its goals, stop
            if drone.goals_reached == len(drone.flights):
                drone.finished = True
                return
            # Set the goal to the next flight's starting position
            drone.goal = np.array(drone.goals[drone.goals_reached])
            drone.current_wait_time = drone.plan[drone.goals_reached][1]
            self.direction = np.subtract(drone.goal, drone.position)

        else:
            if np.linalg.norm(direction) != 0:
                new_direction = direction / np.linalg.norm(direction)
                drone.direction = new_direction
            else:
                drone.direction = direction

            # Move the drone a fixed distance in that direction
            potential_pos = drone.position + drone.direction * (DISTANCE_STEP * self.resolution_factor)
            if potential_pos[0] < 0 or potential_pos[0] > self.grid_size-1 or potential_pos[1] < 0 or potential_pos[1] > self.grid_size-1:
                # print("Drone out of bounds")
                drone.positions.append(drone.position.tolist())

            else:
                drone.position = drone.position + drone.direction * (DISTANCE_STEP * self.resolution_factor)
                drone.positions.append(drone.position.tolist())
                # print(" Direction: ", drone.direction, "Position: ", drone.position, "Goal: ", drone.goal)

   


    def discretise_flight_paths(self, drones, from_time=None, to_time=None):
        drone_positions = []

        if to_time is None:
            to_time = max(drone.flights[-1].finish_time for drone in drones)
        if from_time is None:
            from_time = min(drone.flights[0].start_time for drone in drones)

        num_positions = int(math.ceil((to_time - from_time) / TIME_STEP)) + 1

        for drone in drones:
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
                    print("None position detected")
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