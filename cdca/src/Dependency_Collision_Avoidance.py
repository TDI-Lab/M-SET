from collections import defaultdict
import math
import networkx as nx
import matplotlib.pyplot as plt
from .Collision_Strategy import Collision_Strategy
from .Swarm_Constants import MINIMUM_DISTANCE


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
        hierachy = self.build_execution_hierarchy(dependency_tree)
        self.visualize_dependency_tree(dependency_tree)
        self.augment_plans(drones, hierachy)
        

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

                    # Add dependency on earlier flight based on drone_flag
                    if drone_flag == 2:
                        dependency_tree[flight2.flight_node_index].append(flight1.flight_node_index) #Make flight2 dependent on flight1
                    elif drone_flag == 1:
                        dependency_tree[flight1.flight_node_index].append(flight2.flight_node_index) #Make flight1 dependent on flight2

        # Now make sure any non-first flight is dependent in its drones previous flight if not dependent on another flight
        for i, flight in enumerate(self.flight_nodes):
            if flight.flight_index != len(drones[flight.drone_index].flights) - 1:
                if len(dependency_tree[i+1]) == 0:
                    
                    dependency_tree[i+1].append(i)
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
                # There are cycles in the dependency tree
                return None

            execution_hierarchy.append(current_row)

            # Remove executed nodes from remaining_nodes
            remaining_nodes.difference_update(current_row)

        print("Execution hierarchy: ", execution_hierarchy)
        return execution_hierarchy
    
    def augment_plans(self, drones, execution_hierarchy):
        # Augment the plans of drones based on the execution hierarchy.
        for row in execution_hierarchy:
            longest = 0
            for node in row: # Find longest duration flight in row
                if self.flight_nodes[node].flight.duration > longest:
                    longest = self.flight_nodes[node].flight.duration
            for node in row:    # Make all flights in same schedule to have same duration
                if (self.flight_nodes[node].flight.duration < longest):
                    drones[self.flight_nodes[node].drone_index].augment_plan(self.flight_nodes[node].flight_index, longest)

        dependency_tree = self.build_dependency_tree(drones)
        if dependency_tree is None:
            return False
        self.build_execution_hierarchy(dependency_tree)
        self.visualize_dependency_tree(dependency_tree)
                
    def check_collision(self, flight1, flight2):
        # Check for collision between two flights.
        if self.same_air_time(flight1, flight2):
            collision_flag, drone_flag = self.compare_coordinates(flight1, flight2)
            if collision_flag:
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
                if (flight1.start_time <= flight2.start_time):     # Output earlier flight
                    return True, 2
                else: 
                    return True, 1
        
        return False, None
