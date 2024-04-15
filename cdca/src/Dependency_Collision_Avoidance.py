from collections import defaultdict
import math
import networkx as nx
import matplotlib.pyplot as plt
from .Collision_Strategy import Collision_Strategy
from .Swarm_Constants import *
import numpy as np

FLIGHT_1 = 11
FLIGHT_2 = 22

class FlightNode:
    def __init__(self, drone_index, flight_index, flight_node_index, flight):
        self.drone_index = drone_index
        self.flight_index = flight_index
        self.flight_node_index = flight_node_index
        self.flight = flight

class Dependency_Collision_Avoidance(Collision_Strategy):
    """
    A collision avoidance strategy that creates a dependency tree, describing the order of execution of the drones.

    """

    def __init__(self, visualise=False):
        """
        Initializes a Dependency_Collision_Avoidance object.

        Args:
            visualise (bool): Flag indicating whether to visualize the dependency tree. Default is False.
        """
        flight_nodes = []
        self.visualise_dependencies = visualise

    def detect_potential_collisions(self, drones):
        """
        Detects potential collisions between flights of the given drones.

        Args:
            drones (list): List of Drone objects.

        Returns:
            bool: True if collision avoidance is successful (no collisions remaining), False otherwise.
        """
        # Detect potential collisions by iterating through the flights.
        dependency_tree = self.build_dependency_tree(drones)
   
        hierachy = self.build_execution_hierarchy(dependency_tree)

        if self.visualise_dependencies: 
            self.visualize_dependency_tree(dependency_tree)

        if hierachy is None:
            result = self.augment_plans_without_dependency(drones)
        else:
            result = self.augment_plans_with_dependency(drones, hierachy)

        # if result:
        #     print("Plans augemented successfully. No collisions remaining.")
        # else:
        #     print("Collision avoidance failed. Collisions still detected")
       
        if self.visualise_dependencies: 
            dependency_tree = self.build_dependency_tree(drones)
            hierachy = self.build_execution_hierarchy(dependency_tree)
            self.visualize_dependency_tree(dependency_tree)
        return not result
        
    def update_flight_nodes(self, drones):
        """
        Updates the flight nodes based on the given drones. This is necessary after updating the flight plans.

        Args:
            drones (list): List of Drone objects.
        """
        node_index = 0
        self.flight_nodes = []
        for i, drone in enumerate(drones):
            for j, flight in enumerate(drone.flights):
                self.flight_nodes.append(FlightNode(i, j, node_index, flight))
                node_index += 1
 
        
    def build_dependency_tree(self, drones):
        """
        Builds a dependency tree based on the flights of the given drones.

        Args:
            drones (list): List of Drone objects.

        Returns:
            dict: Dependency tree where the keys are flight node indices and the values are lists of dependent flight node indices.
        """
        # Build a dependency tree based on flights.
        node_index = 0
        self.flight_nodes = []
        for i, drone in enumerate(drones):
            for j, flight in enumerate(drone.flights):
                self.flight_nodes.append(FlightNode(i, j, node_index, flight))
                node_index += 1

        # Make nodes in the dependency tree for each flight
        dependency_tree = {i: [] for i in range(len(self.flight_nodes))}
    
        for j, flight1 in enumerate(self.flight_nodes):
            for k in range(j + 1, len(self.flight_nodes)):
                flight2 = self.flight_nodes[k]

                # Skip flights with same drone
                if flight1.drone_index == flight2.drone_index:
                    continue


                collision, flag = self.do_flights_collide(drones, flight1, flight2)
                if collision:
                    if flag == PARALLEL_COLLISION:
                        dependency_tree[flight2.flight_node_index].append([flight1.flight_node_index, flag]) #only add it to one to avoid cycles in tree
                        continue

                        # For now, all solutions to remaining collision types are the same
                    elif flag == DEST_OCCUPIED_COLLISION:
                        if flight1.flight.flight_path[-1] == flight2.flight.flight_path[0]:
                            dependency_tree[flight2.flight_node_index].append([flight1.flight_node_index, flag])
                        elif flight2.flight.flight_path[-1] == flight1.flight.flight_path[0]:
                            dependency_tree[flight1.flight_node_index].append([flight2.flight_node_index, flag])
                        else:
                            print("Destination occupied collision detected, but drones are not at the same position")

                        # Get earlier flight
                    elif flag == CROSS_COLLISION:
                        if flight1.flight.start_time < flight2.flight.start_time:
                            dependency_tree[flight1.flight_node_index].append([flight2.flight_node_index, flag])

                        elif flight1.flight.start_time > flight2.flight.start_time:
                            dependency_tree[flight2.flight_node_index].append([flight1.flight_node_index, flag])

                        else:
                            # Make earlier flight be drone with lowest index, for priority
                            earlier_flight_index = min(flight1.flight_node_index, flight2.flight_node_index)
                            later_flight_index = max(flight1.flight_node_index, flight2.flight_node_index)
                            dependency_tree[self.flight_nodes[earlier_flight_index].flight_node_index].append([self.flight_nodes[later_flight_index].flight_node_index, flag])
                    elif flag == SAME_FINAL_POSITIONS_COLLISION:
                        # Let flight that finishes last be dependent on the other
                        if flight1.flight.finish_time < flight2.flight.finish_time:
                            dependency_tree[flight1.flight_node_index].append([flight2.flight_node_index, flag])
                        elif flight1.flight.finish_time > flight2.flight.finish_time:
                            dependency_tree[flight2.flight_node_index].append([flight1.flight_node_index, flag])
                        else:
                            # Make earlier flight be drone with lowest index, for priority
                            earlier_flight_index = min(flight1.flight_node_index, flight2.flight_node_index)
                            later_flight_index = max(flight1.flight_node_index, flight2.flight_node_index)
                            dependency_tree[self.flight_nodes[earlier_flight_index].flight_node_index].append([self.flight_nodes[later_flight_index].flight_node_index, flag])
                        

                        
        # Now make sure any non-first flight is dependent in its drones previous flight if not dependent on another flight
        for i, flight in enumerate(self.flight_nodes):
            if flight.flight_index != len(drones[flight.drone_index].flights) - 1:
                if len(dependency_tree[i+1]) == 0:
                    
                    dependency_tree[i+1].append([i, -1])
        # print("Dependency tree: ", dependency_tree)
        return dependency_tree

    # Function to get the order of execution of the drones
    # This function should return a list where the first dimension is all the flights that can be executed in parallel at that step
    # The second dimension is [flight_node_index, [dependencies, collision flag]]
    # dependency_tree can be disconnected, and the flow of dependencies is not always unidirectional
    #
    def build_execution_hierarchy(self, dependency_tree):
        """
        Builds the execution hierarchy based on the dependency tree.

        Args:
            dependency_tree (dict): Dependency tree where the keys are flight node indices and the values are lists of dependent flight node indices.

        Returns:
            list: Execution hierarchy where each element represents a row of flights that can be executed in parallel, along with their dependencies and collision flags.
        """
        execution_hierarchy = []
        remaining_nodes = set(dependency_tree.keys())

        while remaining_nodes:
            current_row = [node for node in remaining_nodes if not any(dependency [0] in remaining_nodes for dependency in dependency_tree[node])]
            if not current_row:
                # print("Error. Cycles in dependency tree. Cannot resolve execution order.")
                # There are cycles in the dependency tree
                return None
            
            row_with_flags = []
            for node in current_row:
                 row_with_flags.append([node, dependency_tree[node]])

            execution_hierarchy.append(row_with_flags)

            # Remove executed nodes from remaining_nodes
            remaining_nodes.difference_update(current_row)

        # print("Execution hierarchy: ", execution_hierarchy)
        return execution_hierarchy
    
    def parallel_flights(self, flight1, flight2):
        """
        Checks if two flights are parallel.

        Args:
            flight1 (FlightNode): First flight node.
            flight2 (FlightNode): Second flight node.

        Returns:
            bool: True if the flights are parallel, False otherwise.
        """
        # Check if x coordinates are not changing
        if flight1.flight_path[0][0] == flight1.flight_path[-1][0] and flight2.flight_path[0][0] == flight2.flight_path[-1][0]:
            return True # Both horizontal, so parallel

        # Check if y coordinates are not changing
        if flight1.flight_path[0][1] == flight1.flight_path[-1][1] and flight2.flight_path[0][1] == flight2.flight_path[-1][1]:
            return True # Both vertical, so parallel

        if flight1.flight_path[-1][0] - flight1.flight_path[0][0] == 0 or flight2.flight_path[-1][0] - flight2.flight_path[0][0] == 0:
            return False # We know they're not the same / parallel from previous checks
        m1 = (flight1.flight_path[-1][1] - flight1.flight_path[0][1]) / (flight1.flight_path[-1][0] - flight1.flight_path[0][0])
        m2 = (flight2.flight_path[-1][1] - flight2.flight_path[0][1]) / (flight2.flight_path[-1][0] - flight2.flight_path[0][0])

        if m1 * m2 == -1: # if product is -1, they are perpendicular
            return False
        # Check for almost parallel paths with similar intercepts
        if abs(m1 - m2) < 1e-6: 
            # Handle the case of almost parallel paths
            return True
        else:
            return False

   

    def augment_plans_with_dependency(self, drones, execution_hierarchy):
        """
        Augments the flight plans of the drones based on the execution hierarchy.

        Args:
            drones (list): List of Drone objects.
            execution_hierarchy (list): Execution hierarchy where each element represents a row of flights that can be executed in parallel, along with their dependencies and collision flags.

        Returns:
            bool: True if the plans are successfully augmented (no collisions remaining), False otherwise.
        """
        row_index = 0
        for row in execution_hierarchy:

            for node, (dependent) in execution_hierarchy[row_index]:    # Make all flights in same schedule to have same duration
                    wait = 0
                    while (wait < COLLISION_AVOIDANCE_LIMIT):

                        collision, flag, other_flight_node = self.does_flight_collide(drones, self.flight_nodes[node].flight_node_index)
                        if collision:
                            if flag == PARALLEL_COLLISION:

                                new_position = self.get_new_position( self.flight_nodes[node], other_flight_node)
                                # print("new position: ", new_position)
                                drones[other_flight_node.drone_index].plan.insert(other_flight_node.flight_index + 1, [new_position, 0])
                                drones[other_flight_node.drone_index].augment_plan(other_flight_node.flight_index, 0)

                            elif flag == DEST_OCCUPIED_COLLISION:
                                drones[other_flight_node.drone_index].augment_plan(other_flight_node.flight_index, -1)

                                # print(f"Destination occupied. ... Drone {self.flight_nodes[node].drone_index} cannot reach destination\n")
                                # print(f"Augmenting plan for drone {other_flight_node.drone_index} to move for 1 second earlier\n")
                                
                                
                            elif flag == CROSS_COLLISION:
                                drones[self.flight_nodes[node].drone_index].augment_plan(self.flight_nodes[node].flight_index, 1)
                                # print(f"Cross collision detected. Augmenting plan for drone {self.flight_nodes[node].drone_index} to wait for 1 more second\n")

                            elif flag == SAME_FINAL_POSITIONS_COLLISION:
                                # print(f"Same final positions collision detected.\n")
                                # get plan that finishes last to go to different position (one already on its path)
                                if self.flight_nodes[node].flight.finish_time <= other_flight_node.flight.finish_time:
                                    self.adjust_plan_same_final_position(drones, self.flight_nodes[node])
                                else:
                                    self.adjust_plan_same_final_position(drones, other_flight_node)
                                   

                                
                        self.update_flight_nodes(drones)
                        wait += 1
                   
            row_index += 1

        result = self.collisions(self.discretise_flight_paths(drones))
                                 
        return not result

    def augment_plans_without_dependency(self, drones):
        """
        Augments the flight plans of the drones based on time they start.

        Args:
            drones (list): List of Drone objects.
            execution_hierarchy (list): Execution hierarchy where each element represents a row of flights that can be executed in parallel, along with their dependencies and collision flags.

        Returns:
            bool: True if the plans are successfully augmented (no collisions remaining), False otherwise.
        """

        # make a copy of self.flight_nodes and sort it based on start time
        sorted_flight_nodes = sorted(self.flight_nodes, key=lambda x: x.flight.start_time)
        
        row_index = 0
        for node in sorted_flight_nodes:

            wait = 0
            while (wait < COLLISION_AVOIDANCE_LIMIT):

                collision, flag, other_flight_node = self.does_flight_collide(drones, node.flight_node_index)
                if collision:
                    if flag == PARALLEL_COLLISION:

                        new_position = self.get_new_position( node, other_flight_node)
                        # print("new position: ", new_position)
                        drones[other_flight_node.drone_index].plan.insert(other_flight_node.flight_index + 1, [new_position, 0])
                        drones[other_flight_node.drone_index].augment_plan(other_flight_node.flight_index, 0)

                    elif flag == DEST_OCCUPIED_COLLISION:
                        drones[other_flight_node.drone_index].augment_plan(other_flight_node.flight_index, -1)

                        # print(f"Destination occupied. ... Drone {self.flight_nodes[node].drone_index} cannot reach destination\n")
                        # print(f"Augmenting plan for drone {other_flight_node.drone_index} to move for 1 second earlier\n")
                        
                        
                    elif flag == CROSS_COLLISION:
                        drones[node.drone_index].augment_plan(node.flight_index, 1)
                        # print(f"Cross collision detected. Augmenting plan for drone {node.drone_index} to wait for 1 more second\n")

                    elif flag == SAME_FINAL_POSITIONS_COLLISION:
                        # print(f"Same final positions collision detected.\n")
                        # get plan that finishes last to go to different position (one already on its path)
                        if node.flight.finish_time <= other_flight_node.flight.finish_time:
                            self.adjust_plan_same_final_position(drones, node)
                        else:
                            self.adjust_plan_same_final_position(drones, other_flight_node)
                                   

                                
                self.update_flight_nodes(drones)
                wait += 1
                   
            row_index += 1

        result = self.collisions(self.discretise_flight_paths(drones))
                                 
        return not result


    def visualize_dependency_tree(self, dependency_tree):
        """
        Visualizes the dependency tree using NetworkX and Matplotlib.

        Args:
            dependency_tree (dict): A dictionary representing the dependency tree, where the keys are parent nodes and the values are lists of child nodes.

        Returns:
            None
        """
        collision_types = {
            1: "Cross Collision",
            2: "Parallel Collision",
            3: "Destination Occupied Collision",
            4: "Same Final Position Collision",
            -1: "Next flight"
        }
        G = nx.DiGraph()
        edge_labels = {}

        for parent, children in dependency_tree.items():
            G.add_node(parent, label=f'Drone: {self.flight_nodes[parent].drone_index}, Flight: {self.flight_nodes[parent].flight_index}')
            for child, flag in children:
                G.add_edge(child, parent)  # Edges represent dependency
                edge_labels[(child, parent)] = str(collision_types[flag])
                if flag == 2:  # If it's a parallel collision
                    G.add_edge(parent, child)  # Add an edge in the opposite direction
                    edge_labels[(parent, child)] = str(collision_types[flag])

        pos = nx.spring_layout(G, k=2)
        labels = nx.get_node_attributes(G, 'label')
        nx.draw(G, pos, with_labels=True, labels=labels, arrows=True, node_size=700, node_color='skyblue', font_size=8, font_color='black', font_weight='bold')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red', font_size=6)

        plt.title("Dependency Tree Visualization")
        plt.show()

    def discretise_flight_paths(self, drones, from_time=None, to_time=None):
        """
        Discretises the flight paths of the given drones within the specified time range.

        Args:
            drones (list): A list of Drone objects.
            from_time (float, optional): The start time of the time range. If not provided, the start time of the first flight will be used.
            to_time (float, optional): The end time of the time range. If not provided, the finish time of the last flight will be used.

        Returns:
            list: A list of lists, where each inner list represents the positions of a drone at different time steps within the specified time range.
                  If any position is None, it indicates an error in the discretisation process.
                  
        """
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


  
    

    def collisions(self, drone_positions):
        """
        Detects collisions between drones based on their discretised positions.

        Args:
            drone_positions (list): List of lists, where each inner list represents the positions of a drone at different time steps.

        Returns:
            bool: True if a collision is detected, False otherwise.
        """
        # Check for collisions between drones
        for i in range(len(drone_positions)):
            for j in range(len(drone_positions)):
                if i == j:
                    continue
                for k in range(len(drone_positions[i])):
                    # print("Distance: ", math.dist(drone_positions[i][k], drone_positions[j][k]))

                    if math.dist(drone_positions[i][k], drone_positions[j][k]) <= MINIMUM_DISTANCE * 1.5:
                        return True
        return False
    
    
        
    
    # Check if a flight collides with any other drone/flights
    def does_flight_collide(self, drones, flight_node_index):
        """
        Detects collisions between a flight with any other flights based on their drone positions.

        Args:
            drones (list): List of all (or a subset) of the drones.
            flight_node_index (int): Index of the flight node to check for collisions.

        Returns:

            tuple: A tuple containing a boolean value indicating if a collision is detected, a string representing the type of collision detected, and the flight node with which the collision occurs.
        """
        for flight_node in self.flight_nodes:
              if flight_node.flight_node_index == flight_node_index or self.flight_nodes[flight_node_index].drone_index == flight_node.drone_index:
                continue
              collision, flag = self.do_flights_collide(drones, self.flight_nodes[flight_node_index], flight_node)
              if collision:
                return True, flag, flight_node
        return False, None, None


    
    # Check if two flights collide
    # Feed in flight nodes instead of flights for more information about flights
    def do_flights_collide(self, drones, flight_node_1, flight_node_2):
        """
        Determines if two flights collide based on their drone positions.

        Args:
            drones (list): List of all drones.
            flight_node_1 (FlightNode): Flight node containing information about the first flight.
            flight_node_2 (FlightNode): Flight node containing information about the second flight.

        Returns:
            tuple: A tuple containing a boolean value indicating if a collision is detected and a string representing the type of collision detected.
        """
        start_time = min(flight_node_1.flight.start_time, flight_node_2.flight.start_time)
        finish_time = flight_node_1.flight.finish_time
        drones_subset = [drones[flight_node_1.drone_index], drones[flight_node_2.drone_index]]
        drone_positions = self.discretise_flight_paths(drones_subset, start_time, finish_time)

        collision_index = None
        for i in range(len(drone_positions[0])):
            if math.dist(drone_positions[0][i],drone_positions[1][i]) <= MINIMUM_DISTANCE * 1.5:
                collision_index = i
                break

        if collision_index is not None:
            if self.parallel_flights(flight_node_1.flight, flight_node_2.flight):
                if flight_node_1.flight.flight_path[0] != flight_node_2.flight.flight_path[0]:
                    # print("Parallel Collision Detected")
                    return True, PARALLEL_COLLISION
            
            # If both drones have no future positions, they will collide at the same final position
            if (flight_node_1.flight.flight_path[-1] == flight_node_2.flight.flight_path[-1]):
                if (not self.will_drone_move(drones[flight_node_2.drone_index], flight_node_2.flight.finish_time)) and (not self.will_drone_move(drones[flight_node_1.drone_index], flight_node_1.flight.finish_time)):
                    # print("Same final positions collision detected")
                    return True, SAME_FINAL_POSITIONS_COLLISION
            
            # Check if final collision point is end of flight path, as this distinguishes it from cross collision
            if (flight_node_2.flight.flight_path[0] == flight_node_1.flight.flight_path[-1]):
                # print("Destination Occupied Collision Detected")
                return True, DEST_OCCUPIED_COLLISION
            
            
            # print("Cross Collision Detected") #assumed if not other collision type
            return True, CROSS_COLLISION
            

        return False, None
    
    def will_drone_move(self, drone, after_time):
        """
        Check if the drone will move after the given time. Useful to detect if a drone should wait to move to that position or not.

        Parameters:
        - drone: The drone object to check.
        - after_time: The time to check if the drone will move after.

        Returns:
        - True if the drone will move after the given time, False otherwise.
        """
        for flight in drone.flights:
            if flight.start_time <= after_time < flight.finish_time:
                return True
        return False
    
    def get_new_position(self, flight_node_1, flight_node_2):
        """
        Calculates the new destination for a flight node to avoid a parallel collision with another flight node, along the line perpendicular to the line connecting the two flights.

        Parameters:
            flight_node_1 (FlightNode): The first flight node.
            flight_node_2 (FlightNode): The second flight node.

        Returns:
            list: The new destination as a list of coordinates [x, y].
        """

        flight1 = np.array(flight_node_1.flight.flight_path[0])
        flight2 = np.array(flight_node_2.flight.flight_path[0])

        collision_point = (flight1 + flight2) / 2  # Midpoint of the two flights

        # Calculate the direction vector of the line connecting the two flights
        direction = flight2 - flight1

        # Calculate the perpendicular direction vector
        perpendicular_direction = np.array([-direction[1], direction[0]])
        normalized_direction = perpendicular_direction / np.linalg.norm(perpendicular_direction)

        distance_apart = MINIMUM_DISTANCE * 2

        # Calculate new destination by moving the collision point MINIMUM_DISTANCE in the perpendicular direction
        new_destination = collision_point + normalized_direction * distance_apart

        return new_destination.tolist()

    def adjust_plan_same_final_position(self, drones, flight_node):
        """
        Adjusts the flight plan of a drone that has the same final position as the previous flight node.
        It calculates a new position that is 1.5 times the minimum distance away from the final position in the direction of the start position.
        This adjustment is made to allow other drones to reach their goal if it coincides with this drone's start position.

        Args:
            drones (list): List of drones.
            flight_node (FlightNode): The flight node for which the plan needs to be adjusted.

        """
        start_position = np.array(flight_node.flight.flight_path[0])
        end_position = np.array(flight_node.flight.flight_path[-1])

        # Calculate the direction vector
        direction = start_position - end_position
        direction = direction / np.linalg.norm(direction)

        # Calculate the position 1.5*minimum distance away from the final position in the direction of the start
        position = end_position + (1.5 * MINIMUM_DISTANCE) * direction

        position = position.tolist()

        # Update plan
        drones[flight_node.drone_index].plan[flight_node.flight_index + 1][0] = position
        drones[flight_node.drone_index].augment_plan(flight_node.flight_index, 0)






    

    