
import math

from matplotlib import animation, pyplot as plt
from src.Drone import Drone
from src.Flight import Flight
from src.Collision_Strategy import Collision_Strategy
import numpy as np
from src.Swarm_Constants import *
from mpl_toolkits.mplot3d import Axes3D

class PF_Drone(Drone):
    def __init__(self, drone, resolution_factor):
        self.original_drone = drone
        super().__init__( drone.plan)
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

        for i in range(len(flight_path) - 1):
            destination = list(self.grid_to_drone([flight_path[i + 1][0], flight_path[i + 1][1]]))
            if float(destination[0]) == float(drone.plan[goal][0][0]) and float(destination[1]) == float(drone.plan[goal][0][1]):
                continue
            elif float(destination[0]) == float(drone.plan[goal+1][0][0]) and float(destination[1]) == float(drone.plan[goal+1][0][1]):
                goal += 1
                continue
                
            else:
                drone.original_drone.plan.insert(goal + index, [destination, 0])

                index += 1

        drone.original_drone.augment_plan(0, 0)



class Potential_Fields_Collision_Avoidance(Collision_Strategy):
    def __init__(self):
        self.resolution_factor = 2
        self.grid_size = GRID_SIZE * self.resolution_factor

    def detect_potential_collisions(self, drones):
        # Detect potential collisions by iterating through the drones and their flights.
        drone_index = 0
        collision = False
        drone_positions = self.discretise_flight_paths(drones)
        for i, drone in enumerate(drones):

            collision_flag = self.does_drone_collide( i, drone_positions)
            if collision_flag:
                collision = True
                break

          
        
        if collision:
            self.potential_fields(drones)
        return False
    
    def potential_fields(self, drones):
        # Initialise PF_Drone objects
        drones = [PF_Drone(drone, self.resolution_factor) for drone in drones]
        
        drones_not_done = len(drones)

        self.time_step = 0
        pfs_per_drone = [[] for _ in drones]
        count = 0
        drones_done = []
        while (drones_not_done) > 0 and count < 100:
            count += 1

            for i, drone in enumerate(drones):

                # Skip drones that have reached all their goals
                if drone.finished:
                    continue

                if drone.current_wait_time > 0:
                    drone.current_wait_time -= TIME_STEP
                    drone.positions.append(drone.position)
                    continue
                    
                potential_field = self.calculate_potential_field(drone, drones)

                # Add potential field to list for visualisation
                pfs_per_drone[i].append(potential_field)

                self.adjust_drone_path(drone, potential_field)
            
            self.time_step += TIME_STEP
            self.time_step = round(self.time_step, 2)
            print("Time step: ", self.time_step)
        for i, drone in enumerate(drones):
            # self.animate_potential_fields(pfs_per_drone[i], drone, i)
            print("num of pfs: ", len(pfs_per_drone[i]))
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
        # self.plot_vector_field(potential_field)
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
        maximum = MINIMUM_DISTANCE * self.resolution_factor * 2
        mask = (distances >= minimum) & (distances <= maximum)
        # Apply the mask to the vectors and divide by the square of the distances
        vectors[0] = vectors[0] * mask / (distances**2 + 1e-9)
        vectors[1] = vectors[1] * mask / (distances**2 + 1e-9)
        # Scale the vectors by a factor that is inversely proportional to the distance
        scale_factor = maximum / (distances + 1e-9)
        vectors[0] *= scale_factor
        vectors[1] *= scale_factor
        # Calculate and print the magnitudes of the vectors
        magnitudes = np.sqrt(vectors[0]**2 + vectors[1]**2)
        # print("Magnitudes: ", magnitudes)
        # self.plot_vector_field(vectors)

        # print("vectors shape: ", vectors[0].shape, vectors[1].shape)
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
        
        # Calculate and print the magnitudes of the vectors
        magnitudes = np.sqrt(unit_vectors[0]**2 + unit_vectors[1]**2)
        # print("Magnitudes: ", magnitudes)
        # self.plot_vector_field(unit_vectors)

        # print("goal unit vectors shape: ", unit_vectors[0].shape, unit_vectors[1].shape)

        return unit_vectors


    def plot_vector_field(self, unit_vectors, drone):
        x = np.linspace(0, self.grid_size, self.grid_size)
        y = np.linspace(0, self.grid_size, self.grid_size)
        X, Y = np.meshgrid(x, y)

        fig, ax = plt.subplots()
        # print("vector[0]: ", unit_vectors[0].shape, "vector[1]: ", unit_vectors[1].shape)
        # The unit_vectors list contains two arrays: one for the x-components of the vectors, and one for the y-components
        # Select the first set of vectors
        U = unit_vectors[0][0]
        V = unit_vectors[1][0]

        Q = ax.quiver(X, Y, U, V)
        drone_position, = ax.plot([], [], 'ro')  # plot drone as a red dot

        def update(num, Q, drone_position):
            """updates the horizontal and vertical vector components by a
            fixed increment on each frame
            """
            U = np.cos(num / 100.) * unit_vectors[0]
            V = np.sin(num / 100.) * unit_vectors[1]

            Q.set_UVC(U, V)

            # update drone's position
            drone_position.set_data(drone.position[0], drone.position[1])

            return Q, drone_position

        # you need to set blit=False, or the first set of arrows never gets
        # cleared on subsequent frames
        ani = animation.FuncAnimation(fig, update, fargs=(Q, drone_position),
                                      interval=10, blit=False)
        plt.show()

    def visualize_gradient(self, scalar_field, gradient):
        x = np.linspace(0, self.grid_size, self.grid_size)
        y = np.linspace(0, self.grid_size, self.grid_size)
        X, Y = np.meshgrid(x, y)

        # The gradient list contains two arrays: one for the x-components of the vectors, and one for the y-components
        U = gradient[0]
        V = gradient[1]

        plt.figure(figsize=(10, 10))
        plt.quiver(X, Y, U, V, color='r')

        # Also plot the scalar field for reference
        plt.contourf(X, Y, scalar_field, alpha=0.5)
        plt.colorbar(label='Distance to goal')
        plt.title('Gradient of Scalar Field (Distance to Goal)')
        plt.show()
        
   

    def animate_gradient(self, potential_fields, drone):
        fig = plt.figure(figsize=(7,7))

        # Create a 3D subplot for the surface plot
        ax1 = fig.add_subplot(2, 1, 1, projection='3d')

        # Create a 2D subplot for the quiver plot
        ax2 = fig.add_subplot(2, 1, 2)

        # Create a grid of x, y coordinates
        y, x = np.mgrid[0:potential_fields[0].shape[0], 0:potential_fields[0].shape[1]]

        # Initial surface plot and quiver plot
        surf = ax1.plot_surface(x, y, potential_fields[0], cmap='viridis')
        quiver = ax2.quiver(x, y, *np.gradient(-potential_fields[0], edge_order=2), color='r')
        ax2.scatter(*(drone.positions[0]), color='b')
        def update(i):
            # Update surface plot
            ax1.clear()
            ax1.plot_surface(x, y, potential_fields[i], cmap='viridis')
            ax1.scatter(*(drone.positions[i]), color='b')

            ax1.set_title('3D Visualization of Potential Field')

            # Update quiver plot
            ax2.clear()
            ax2.quiver(x, y, potential_fields[i], color='r')
            ax2.scatter(*drone.positions[i], color='b')
            ax2.set_title('Gradient of Potential Field')

        ani = animation.FuncAnimation(fig, update, frames=len(potential_fields), repeat=False)

        plt.tight_layout()
        plt.show()

    def animate_vector_fields(self, vector_fields, drone):
        fig = plt.figure(figsize=(7,7))

        # Create a 2D subplot for the quiver plot
        ax = fig.add_subplot(1, 1, 1)

        # Create a grid of x, y coordinates
        y, x = np.mgrid[0:vector_fields[0][0].shape[0], 0:vector_fields[0][0].shape[1]]

        # Initial quiver plot
        quiver = ax.quiver(x, y, vector_fields[0][0], vector_fields[0][1])
        ax.scatter(*(drone.positions[0]), color='b')

        def update(i):
            # Update quiver plot
            ax.clear()
            ax.quiver(x, y, vector_fields[i][0], vector_fields[i][1])
            ax.scatter(*drone.positions[i], color='b')
            ax.set_title('Vector Field')

        ani = animation.FuncAnimation(fig, update, frames=len(vector_fields), repeat=False)

        plt.tight_layout()
        plt.show()
    def adjust_drone_path(self, drone, potential_field):
         # Calculate the gradient of the potential field
        # Get the gradient at the drone's position
        grid_drone_position = (drone.position).astype(int)
        vector_at_drone = potential_field[:, grid_drone_position[1], grid_drone_position[0]]
        print("Vector at drone's position: ", vector_at_drone)
        direction = vector_at_drone
        # The direction of the lowest potential is the negative of the gradient
        # Add a small random component to the direction
        # direction += np.random.normal(scale=0.2, size=direction.shape)
        # print("Potential field at drone's position: ", potential_field[drone_position_int[1], drone_position_int[0]])
        # print("Computed gradient at drone's position: ", gradient_at_drone)
        # Add a leftward bias to the direction
        # direction += np.array([-direction[1], direction[0]]) * 0.1
        # Get distance from the current position to the goal
        distance_to_goal = np.linalg.norm(drone.position - drone.goal)
        # Check if the drone is close enough to the goal
        if distance_to_goal <= (DISTANCE_STEP * self.resolution_factor):
            drone.positions.append(drone.goal)
            # If the drone is close enough to the goal, increment the number of goals reached
            # If the drone has reached all its goals, stop
            if drone.goals_reached >= len(drone.flights)-1:
                drone.finished = True
                return
            else:
                drone.goals_reached += 1
            # Set the goal to the next flight's starting position
            drone.goal = np.array(drone.goals[drone.goals_reached])
            drone.current_wait_time = drone.plan[drone.goals_reached][1]
            self.direction = np.subtract(drone.goal, drone.position)
            print(" Direction: ", drone.direction, "Position: ", drone.position, "Goal: ", drone.goal)

        else:
            if np.linalg.norm(direction) != 0:
                new_direction = direction / np.linalg.norm(direction)
                drone.direction = new_direction
            else:
                drone.direction = direction
            # Normalize the direction vector
            # drone.direction = direction / np.linalg.norm(direction)
            # Move the drone a fixed distance in that direction
            potential_pos = drone.position + drone.direction * (DISTANCE_STEP * self.resolution_factor)
            if potential_pos[0] < 0 or potential_pos[0] > self.grid_size-1 or potential_pos[1] < 0 or potential_pos[1] > self.grid_size-1:
                print("Drone out of bounds")
                drone.positions.append(drone.position.tolist())

            else:
                drone.position = drone.position + drone.direction * (DISTANCE_STEP * self.resolution_factor)
                drone.positions.append(drone.position.tolist())
                print(" Direction: ", drone.direction, "Position: ", drone.position, "Goal: ", drone.goal)

   


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
                first_known_position = flight.flight_path[0]
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