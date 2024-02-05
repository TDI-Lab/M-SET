from .Flight import Flight
from .Swarm_Constants import TIME_DELAY

class Drone:
  # Class for each drone on the testbed. 

  def __init__(self, plan):
    # Initialise drone object with a plan.
    self.plan = plan
    self.create_flights()


  def create_flights(self):
    # Create flights from the plan.
    self.flights = []
    start_time = 0
    for i in range(len(self.plan) - 1):
        start_time += self.plan[i][1]
        new_flight = Flight(start_time, self.plan[i], self.plan[i+1])
        self.flights.append(new_flight)
        start_time += new_flight.duration
    

  def augment_plan(self, index, delay=TIME_DELAY):
    # Add TIME_DELAY to the origin in the collision flight.
    self.plan[index][1] += delay
    self.create_flights()
    

  def print_itinerary(self):
    print("\nDrone plan: ", *self.plan,sep='\n')
    print("\nDrone flights: \n\n")
    flight_index = 0
    for flight in self.flights:
        print("\nFlight ", flight_index, ": \n")
        flight.print_itinerary()
        flight_index += 1
    
