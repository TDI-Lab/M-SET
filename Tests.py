import unittest
from src.Flight import Flight
from src.Drone import Drone
from src.Swarm_Control import Swarm_Control
from src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
from src.Swarm_Constants import TIME_DELAY

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

        self.assertEqual(len(swarm_controller.drones), 2, '')

        self.assertEqual(swarm_controller.drones[0].plan, plans[0], '')
        self.assertEqual(swarm_controller.drones[1].plan, plans[1], '')
    
    def test_detect_potential_collisions(self):
        plans = [[[[1,1], 5], [[9,9], 3]], [[[9,1], 5], [[1,9], 3]]]
        second_drone_source_duration = plans[1][0][1]
        swarm_controller = Swarm_Control(plans, Basic_Collision_Avoidance())
        swarm_controller.detect_potential_collisions()

        self.assertEqual(swarm_controller.drones[1].plan[0][1], second_drone_source_duration + TIME_DELAY, '')



if __name__ == '__main__':
    unittest.main()