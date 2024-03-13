from collections import defaultdict
import math
import networkx as nx
import matplotlib.pyplot as plt
from .Collision_Strategy import Collision_Strategy
from .Swarm_Constants import DISTANCE_STEP, MINIMUM_DISTANCE, PARALLEL_COLLISION, SPEED


class FlightNode:
    def __init__(self, drone_index, flight_index, flight_node_index, flight):
        self.drone_index = drone_index
        self.flight_index = flight_index
        self.flight_node_index = flight_node_index
        self.flight = flight
        

class Dependency_Collision_Avoidance(Collision_Strategy):
    # A collision avoidance strategy that creates a dependency tree.
    flight_nodes = []

    def detect_potential_collisions(self, drones):
        # Detect potential collisions by iterating through the flights.
        dependency_tree = self.build_dependency_tree(drones)
        if dependency_tree is None:
            return False
        if dependency_tree is PARALLEL_COLLISION:
            print("PARALLEL COLLISION DETECTED\nCOLLISION AVOIDANCE aborted")
            return False
        hierachy = self.build_execution_hierarchy(dependency_tree)
        
        self.visualize_dependency_tree(dependency_tree)
        ret = self.augment_plans(drones, hierachy)

        if ret is PARALLEL_COLLISION:
            print("PARALLEL COLLISION DETECTED\nCA aborted")
        drone_positions = self.discretise_flight_paths(drones)
        if (self.collisions(drone_positions)):
            print("COLLISIONS STILL DETECTED AFTER AUGMENTATION")
            # return True
        

    def build_dependency_tree(self, drones):
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

                collision_flag, drone_flag = self.check_collision(flight1.flight, flight2.flight)
                if collision_flag:
                    if drone_flag is PARALLEL_COLLISION:
                        return PARALLEL_COLLISION
                    # Add dependency on earlier flight based on drone_flag
                    if drone_flag == 2:
                        print("Drone flag 2")
                        dependency_tree[flight2.flight_node_index].append(flight1.flight_node_index) #Make flight2 dependent on flight1
                    elif drone_flag == 1:
                        print("Drone flag 1")
                        dependency_tree[flight1.flight_node_index].append(flight2.flight_node_index) #Make flight1 dependent on flight2
                    elif drone_flag == 0:
                        print("Drone flag 0")
                        # If both flights are at the same time, make the one with the higher index dependent on the other (drone priority)
                        if flight1.flight_node_index < flight2.flight_node_index:  
                            dependency_tree[flight2.flight_node_index].append(flight1.flight_node_index)
                        else:
                            dependency_tree[flight1.flight_node_index].append(flight2.flight_node_index)

        # Now make sure any non-first flight is dependent in its drones previous flight if not dependent on another flight
        for i, flight in enumerate(self.flight_nodes):
            if flight.flight_index != len(drones[flight.drone_index].flights) - 1:
                if len(dependency_tree[i+1]) == 0:
                    
                    dependency_tree[i+1].append(i)
        print("Dependency tree: ", dependency_tree)
        return dependency_tree

    # Function to get the order of execution of the drones
    # This function should return a list of all the flights that can be executed in parallel
    # dependency_tree can be disconnected, and the flow of dependencies is not always unidirectional
    def build_execution_hierarchy(self, dependency_tree):
        execution_hierarchy = []
        remaining_nodes = set(dependency_tree.keys())

        while remaining_nodes:
            current_row = [node for node in remaining_nodes if not any(dependency in remaining_nodes for dependency in dependency_tree[node])]
            if not current_row:
                print("Cycles in dependency tree")
                # There are cycles in the dependency tree
                return None

            execution_hierarchy.append(current_row)

            # Remove executed nodes from remaining_nodes
            remaining_nodes.difference_update(current_row)

        print("Execution hierarchy: ", execution_hierarchy)
        return execution_hierarchy
    
    def parallel_flights(self, flight1, flight2):

        # Check if x coordinates are not changing
        if flight1.flight_path[0][0] == flight1.flight_path[-1][0] and flight2.flight_path[0][0] == flight2.flight_path[-1][0]:
            return True

        # Check if y coordinates are not changing

        if flight1.flight_path[0][1] == flight1.flight_path[-1][1] and flight2.flight_path[0][1] == flight2.flight_path[-1][1]:
            return True

        # Calculate gradients and intercepts
        if flight1.flight_path[-1][0] - flight1.flight_path[0][0] == 0 or flight2.flight_path[-1][0] - flight2.flight_path[0][0] == 0:
            return False # We know theyre not the same / parallel from previous checks

        m1 = (flight1.flight_path[-1][1] - flight1.flight_path[0][1]) / (flight1.flight_path[-1][0] - flight1.flight_path[0][0])
        c1 = flight1.flight_path[0][1] - m1 * flight1.flight_path[0][0]


        m2 = (flight2.flight_path[-1][1] - flight2.flight_path[0][1]) / (flight2.flight_path[-1][0] - flight2.flight_path[0][0])
        c2 = flight2.flight_path[0][1] - m2 * flight2.flight_path[0][0]

        # Check for almost parallel paths with similar intercepts
        if abs(m1 - m2) < 1e-6:
            # Handle the case of almost parallel paths
            return True
        else:
            return False

    def get_safe_collision_duration(self, flight1, flight2):
        # Check for division by zero
        if flight1.flight_path[-1][0] == flight1.flight_path[0][0] or flight2.flight_path[-1][0] == flight2.flight_path[0][0]:
            # Handle the special case (e.g., parallel paths)
            return None

        # Calculate slopes and intercepts
        m1 = (flight1.flight_path[-1][1] - flight1.flight_path[0][1]) / (flight1.flight_path[-1][0] - flight1.flight_path[0][0])
        c1 = flight1.flight_path[0][1] - m1 * flight1.flight_path[0][0]

        m2 = (flight2.flight_path[-1][1] - flight2.flight_path[0][1]) / (flight2.flight_path[-1][0] - flight2.flight_path[0][0])
        c2 = flight2.flight_path[0][1] - m2 * flight2.flight_path[0][0]

        # Check for almost parallel paths
        if abs(m1 - m2) < 1e-6:
            # Handle the case of almost parallel paths
            print("Almost parallel paths")
            return PARALLEL_COLLISION

        # Calculate collision point
        x = (c2 - c1) / (m1 - m2)
        y = m1 * x + c1

        # Get the time of collision
        collision_point = [x, y]

        # Calculate distance traveled by flight1 to reach the collision point
        distance_traveled_flight1 = math.dist(flight1.flight_path[0], collision_point)

        # Calculate time to reach MINIMUM_DISTANCE from collision point at constant SPEED
        time_to_collision = distance_traveled_flight1 / SPEED

        # Adjust time for MINIMUM_DISTANCE
        time_to_collision += MINIMUM_DISTANCE / SPEED

        # Check if collision point is within the flight duration
        if flight1.duration < time_to_collision or flight2.duration < time_to_collision:
            # Collision point is not within the flight duration
            return None

        return True


        


    def augment_plans(self, drones, execution_hierarchy):
        # # Augment the plans of drones based on the execution hierarchy.
        # for row in execution_hierarchy:
        #     for flight, i in enumerate(row[:-1]):
        #         # skip if drones next flight is dependent on previous flight
        #         prev_drone = self.flight_nodes[flight].drone_index
        #         curr_drone = self.flight_nodes[row[i+1]].drone_index
        #         if (prev_drone != curr_drone):
        #             time_to_collision = self.get_safe_collision_duration(self.flight_nodes[flight].flight, self.flight_nodes[flight+1].flight)
        #             if time_to_collision is PARALLEL_COLLISION:
        #                 continue
        #             wait = math.ceil(abs(time_to_collision - self.flight_nodes[flight+1].flight.start_time))
                
        #             drones[self.flight_nodes[flight+1].drone_index].augment_plan(self.flight_nodes[flight+1].flight_index, 
        #                                                                         wait)
               # Augment the plans of drones based on the execution hierarchy.
        row_index = 0
        for row in execution_hierarchy[:-1]:
            longest = 0
            for node in row: # Find longest duration flight in row
                if self.flight_nodes[node].flight.duration > longest:
                    longest = self.flight_nodes[node].flight.duration
            for node in execution_hierarchy[row_index+1]:    # Make all flights in same schedule to have same duration
                if (self.flight_nodes[node].flight.duration <= longest):
                    while (self.does_flight_collide(drones, self.flight_nodes[node].drone_index, self.flight_nodes[node].flight_index)):
                        drones[self.flight_nodes[node].drone_index].augment_plan(self.flight_nodes[node].flight_index, 1) 
                        print("Adding extra second...\ns")
                   
            row_index += 1
        dependency_tree = self.build_dependency_tree(drones)
        if dependency_tree is None:
            return False
        elif dependency_tree is PARALLEL_COLLISION:
            return PARALLEL_COLLISION
        self.build_execution_hierarchy(dependency_tree)
        self.visualize_dependency_tree(dependency_tree)


    # def does_flight_collide(self, drone_index, discritised_positions):
    #     for drone in drones:
    #         for other_flight in drone.flights:
    #             # Skip if the same flight
    #             if flight == other_flight:
    #                 continue
    #             if self.get_safe_collision_duration(flight, other_flight):
    #                 return True
                
    #     return False


    def flights_collide(self, flight1, flight2):
        # Check if two flights collide.
        for i in range(flight1.duration):
            for j in range(flight2.duration):
                if (flight1.start_time + i == flight2.start_time + j) and (math.dist(flight1.flight_path[i], flight2.flight_path[j]) < DISTANCE_STEP):
                    return True
        return False  
              
    def check_collision(self, flight1, flight2):
        # Check for collision between two flights.
        if self.same_air_time(flight1, flight2):
            collision_flag, drone_flag = self.compare_coordinates(flight1, flight2)
            if collision_flag:
                
                
                parallel = self.parallel_flights(flight1, flight2)
                if parallel:
                    print("Parallel flights")
                    return True, PARALLEL_COLLISION
                return True, drone_flag
            
        return False, None

    def same_air_time(self, flight1, flight2):
        # Check if two flights are airborne at the same time.
        return (flight1.start_time < flight2.finish_time and flight1.finish_time > flight2.start_time) or \
               (flight2.start_time < flight1.finish_time and flight2.finish_time > flight1.start_time)
  
    def visualize_dependency_tree(self, dependency_tree):
        G = nx.DiGraph()
        i = 0
        for parent, children in dependency_tree.items():
            G.add_node(parent, label=f'Drone: {self.flight_nodes[parent].drone_index}, Flight: {self.flight_nodes[parent].flight_index}')
            for child in children:
                G.add_edge(child, parent)  # Edges represent dependency
            i += 1

        pos = nx.spring_layout(G)
        labels = nx.get_node_attributes(G, 'label')
        nx.draw(G, pos, with_labels=True, labels=labels, arrows=True, node_size=700, node_color='skyblue', font_size=8, font_color='black', font_weight='bold')
        plt.title("Dependency Tree Visualization")
        plt.show()

   
        
    def compare_coordinates(self, flight1, flight2):
        # Check if two flights that are airborne at the same time get too close at any point of time.
        comparison_from = max(flight1.start_time, flight2.start_time)
        comparison_until = min(flight1.finish_time, flight2.finish_time)

        for i in range(comparison_from, comparison_until):
            subjet_flight_index = i - flight1.start_time
            flight_index = i - flight2.start_time
            distance_between_drones = math.dist(flight1.flight_path[subjet_flight_index], flight2.flight_path[flight_index])
            if (distance_between_drones <= MINIMUM_DISTANCE):
                #TODO: replace start_time comparison with time at collision point

                if (flight1.start_time < flight2.start_time):     # Output earlier flight
                    return True, 2
                elif (flight1.start_time > flight2.start_time): 
                    return True, 1
                else: return True, 0
        
        return False, None

    def discretise_flight_paths(self, drones, from_time=0, to_time=math.inf):
        # Discretise the flight paths of the drones per each timestep per drone per flight path
        drone_positions=[] #Each element is a list of drone positions at each timestep

        #earliest start time of each drones first flight
        earliest_start_time = [drone.flights[0].start_time for drone in drones]
        min_start_time = min(earliest_start_time)
        
        for drone in drones:
            positions_for_drone = []
            for i in range(len(drone.flights)):
                
                
                flight = drone.flights[i]
                if flight.start_time > to_time or flight.finish_time < from_time:
                    continue
                if i == 0: #ensure positions at start line up
                    for k in range(flight.start_time - min_start_time):
                        positions_for_drone.insert(0, flight.flight_path[0])

                for j in range(max(flight.start_time, from_time), min(flight.finish_time, to_time)):
                    positions_for_drone.append(flight.flight_path[j - flight.start_time])

                if i < len(drone.flights) - 1: #ensure positions between flights line up
                    next_flight = drone.flights[i + 1]
                    time_difference = next_flight.start_time - flight.finish_time
                    for k in range(time_difference):
                        positions_for_drone.append(next_flight.flight_path[0]) 

            drone_positions.append(positions_for_drone)

        for drone in drone_positions:
            print(drone, "\n")
        return drone_positions
    
    def collisions(self, drone_positions):
        # Check for collisions between drones
        for i in range(len(drone_positions)):
            for j in range(len(drone_positions)):
                if i == j:
                    continue
                for k in range(len(drone_positions[i])):
                    if (j < i and k < len(drone_positions[j])) or (j > i and k < len(drone_positions[i])):
                        break
                    if math.dist(drone_positions[i][k], drone_positions[j][k]) < MINIMUM_DISTANCE:
                        return True
        return False
    
    def collisions_with_drone(self, drone_index, drone_positions):
        # Check for collisions between drones with drone at drone_index
        for i in range(len(drone_positions)):
            if i == drone_index:
                continue
            for k in range(min(len(drone_positions[i]), len(drone_positions[drone_index]))):
                if math.dist(drone_positions[i][k], drone_positions[drone_index][k]) < MINIMUM_DISTANCE:
                    return True
        return False
    

   
    
    def does_flight_collide(self, drones, drone_index, flight_index):
        #get the discretised positions of the drones from the start of the flight to end of the flight
        drone_positions = self.discretise_flight_paths(drones, drones[drone_index].flights[flight_index].start_time)
                                                      #  drones[drone_index].flights[flight_index].finish_time)
        
        if self.collisions_with_drone(drone_index, drone_positions):
            return True
        else:
            return False

 

    