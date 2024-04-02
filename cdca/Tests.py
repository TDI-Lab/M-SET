import unittest
from src.Flight import Flight
from src.Drone import Drone
from src.Swarm_Control import Swarm_Control
from src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
from src.Input_Parser import Input_Parser
from src.Swarm_Constants import TIME_DELAY
from src.Dependency_Collision_Avoidance import Dependency_Collision_Avoidance
from src.Potential_Fields_Collision_Avoidance import Potential_Fields_Collision_Avoidance

class TestFlight(unittest.TestCase):

    def test_flight_object(self):
        # DISTANCE_STEP = 1
        origin = [[1,1], 5]
        destination = [[5,5], 10]
        start_time = 42
        flight = Flight(start_time, origin, destination)
        expectet_flight_path = [[1, 1],
                                [1.7071067811865475, 1.7071067811865475],
                                [2.414213562373095, 2.414213562373095],
                                [3.1213203435596424, 3.1213203435596424],
                                [3.82842712474619, 3.82842712474619],
                                [4.535533905932737, 4.535533905932737],
                                [5, 5]]

        self.assertEqual(len(flight.flight_path), 7, '')
        self.assertEqual(flight.flight_path, expectet_flight_path, 'The flight path is wrong.')


class TestDrone(unittest.TestCase):

    def test_drone_object(self):
        plan = [[[1,1], 5], [[4,4], 10], [[9,9], 3]]
        drone = Drone(plan)

        self.assertEqual(len(drone.flights), 2, '')

        self.assertEqual(drone.flights[0].start_time, plan[0][1], '')
        self.assertEqual(drone.flights[0].flight_path[0], plan[0][0], '')
        self.assertEqual(drone.flights[0].flight_path[-1], plan[1][0], '')

        self.assertEqual(drone.flights[1].start_time, plan[0][1] + drone.flights[0].duration + plan[1][1], '')
        self.assertEqual(drone.flights[1].flight_path[0], plan[1][0], '')
        self.assertEqual(drone.flights[1].flight_path[-1], plan[2][0], '')


class TestSwarmControl(unittest.TestCase):

    def test_swarm_control_object(self):
        plans = [[[[1,1], 5], [[9,9], 3]], [[[9,1], 5], [[1,9], 3]]]
        swarm_controller = Swarm_Control(plans, Basic_Collision_Avoidance())
        swarm_controller.print_itinerary()
        self.assertEqual(len(swarm_controller.drones), 2, '')

        self.assertEqual(swarm_controller.drones[0].plan, plans[0], '')
        self.assertEqual(swarm_controller.drones[1].plan, plans[1], '')
    
    def test_basic_collision_avoidance_cross_collision(self):
        plans = [[[[1,1], 5], [[9,9], 3]], [[[9,1], 5], [[1,9], 3]]]
        second_drone_source_duration = plans[1][0][1]
        swarm_controller = Swarm_Control(plans, Basic_Collision_Avoidance())
        swarm_controller.detect_potential_collisions()

        self.assertEqual(swarm_controller.drones[0].plan[0][1], second_drone_source_duration + TIME_DELAY, '')
    
    def test_basic_collision_avoidance_parallel_collision(self):
        plans = [[[[1,1], 5], [[9,9], 3]], [[[9,9], 5], [[1,1], 3]]]
        swarm_controller = Swarm_Control(plans, Basic_Collision_Avoidance())
        swarm_controller.detect_potential_collisions()

        self.assertEqual(len(swarm_controller.drones[0].plan), 3, '')
    
    def test_basic_collision_avoidance_dest_occupied_collision(self):
        plans = [[[[9,1], 1], [[1,1], 3]], [[[1,1], 20], [[9,9], 3]]]
        swarm_controller = Swarm_Control(plans, Basic_Collision_Avoidance())
        swarm_controller.detect_potential_collisions()

        self.assertEqual(swarm_controller.drones[0].plan[0][1], 15, '')
    
    def test_get_offline_collision_stats_1(self):
        plans = [[[[1,1], 5], [[9,9], 3], [[1,1], 5]], [[[9,1], 5], [[1,9], 3]], [[[9,1], 20], [[1,9], 3]]]
        swarm_controller = Swarm_Control(plans, Basic_Collision_Avoidance())
        result = swarm_controller.get_offline_collision_stats()

        self.assertEqual(result['number_of_collisions'], 4, '')
        self.assertEqual(result['number_of_cross_collisions'], 4, '')
        self.assertEqual(result['number_of_parallel_collisions'], 0, '')
        self.assertEqual(result['number_of_dest_occupied_collisions'], 0, '')
    
    def test_get_offline_collision_stats_2(self):
        plans = [[[[1,1], 20], [[9,9], 3]], [[[9,1], 1], [[1,1], 3]]]
        swarm_controller = Swarm_Control(plans, Basic_Collision_Avoidance())
        result = swarm_controller.get_offline_collision_stats()

        self.assertEqual(result['number_of_collisions'], 1, '')
        self.assertEqual(result['number_of_cross_collisions'], 0, '')
        self.assertEqual(result['number_of_parallel_collisions'], 0, '')
        self.assertEqual(result['number_of_dest_occupied_collisions'], 1, '')

 

    def test_dependency_collision_avoidance_dest_occupied(self):
      
        plans = [[[[9,1], 1], [[1,1], 3]], [[[1,1], 20], [[9,9], 3]]] # dest occupied collision
        
        swarm_controller = Swarm_Control(plans, Dependency_Collision_Avoidance())

        result_before_ca = swarm_controller.get_offline_collision_stats()
        self.assertEqual(result_before_ca['number_of_collisions'], 1, '')

        swarm_controller.detect_potential_collisions()
        # swarm_controller.visualise_swarm()

        result = swarm_controller.get_offline_collision_stats()
        self.assertEqual(result['number_of_collisions'], 0, '')

    def test_dependency_collision_avoidance_parallel_collision(self):
      
        plans = [[[[1,1], 5], [[9,9], 3]], [[[9,9], 5], [[1,1], 3]]] # parallel collision
        
        swarm_controller = Swarm_Control(plans, Dependency_Collision_Avoidance())

        result_before_ca = swarm_controller.get_offline_collision_stats()
        self.assertEqual(result_before_ca['number_of_collisions'], 1, '')

        swarm_controller.detect_potential_collisions()
        # swarm_controller.visualise_swarm()

        result = swarm_controller.get_offline_collision_stats()
        self.assertEqual(result['number_of_collisions'], 0, '')

    def test_dependency_collision_avoidance_final_dest_occupied(self):
      
        plans = [[[[9,1], 1], [[1,1], 3]], [[[1,9], 1], [[1,1], 3]]] # final dest occupied collision
        
        swarm_controller = Swarm_Control(plans, Dependency_Collision_Avoidance())

        result_before_ca = swarm_controller.get_offline_collision_stats()
        self.assertEqual(result_before_ca['number_of_collisions'], 1, '')

        swarm_controller.detect_potential_collisions()
        # swarm_controller.visualise_swarm()

        result = swarm_controller.get_offline_collision_stats()
        self.assertEqual(result['number_of_collisions'], 0, '')

    def test_dependency_collision_avoidance_cross_collision(self):
      
        plans = [[[[1,1], 5], [[9,9], 3]], [[[9,1], 5], [[1,9], 3]]] # cross collision
        
        swarm_controller = Swarm_Control(plans, Dependency_Collision_Avoidance())

        result_before_ca = swarm_controller.get_offline_collision_stats()
        self.assertEqual(result_before_ca['number_of_collisions'], 1, '')

        swarm_controller.detect_potential_collisions()
        # swarm_controller.visualise_swarm()

        result = swarm_controller.get_offline_collision_stats()
        self.assertEqual(result['number_of_collisions'], 0, '')




class TestInputParser(unittest.TestCase):

    def test_parse_input(self):
        example_input = {  1: [(1,1), (1,1), (5,5), (5,5)], 
                           2: [(1,3), (3,1), (2,5), (5,3)], 
                           3: [(1,9), (7,4), (5,1), (5,1)], 
                           4: [(9,9), (2,2), (2,2), (5,7)]  }
        
        expected_plans = [ [ [ [1, 1], 20 ], [ [5, 5], 20 ] ],  
                           [ [ [1,3 ], 10 ], [ [3, 1], 10 ], [ [2, 5], 10 ], [ [5, 3], 10 ] ],
                           [ [ [1, 9], 10 ], [ [7, 4], 10 ], [ [5, 1], 20 ] ], 
                           [ [ [9, 9], 10 ], [ [2, 2], 20 ], [ [5, 7], 10 ] ] ]

        plans = Input_Parser(example_input).parsed_input

        self.assertEqual(plans, expected_plans, '')

if __name__ == '__main__':
    unittest.main()
    #Execution hierarchy:  [[0, 8], [1, 2, 6, 9], [3, 4, 7], [5]]
    # Execution hierarchy:  [[0], [1, 3], [2]]