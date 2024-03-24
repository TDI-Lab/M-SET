import math 
from .Swarm_Constants import DISTANCE_STEP, TIME_STEP

class Flight:
  # Class for drone flights between two cells.

  def __init__(self, start_time, origin, destination):
    # Initialise flight object with start time, origin and destination.
    self.start_time = start_time
    self.calculate_flight_path(origin, destination)
    self.calculate_duration()
    self.finish_time = self.start_time + self.duration


  def calculate_duration(self):
    # Calculate flight duration.
    self.duration = round((len(self.flight_path) - 1) * TIME_STEP, 2)


  def calculate_flight_path(self, origin, destination):
    # Calculate flight coordinates.
    self.distance = math.dist(origin[0], destination[0])
    self.flight_path = [origin[0]]
    
    distance_covered = 0
    while ((distance_covered + DISTANCE_STEP) < self.distance):
        self.flight_path.append(self.get_coordinates(origin[0], destination[0], distance_covered + DISTANCE_STEP))
        distance_covered += DISTANCE_STEP

    self.flight_path.append(destination[0])


  def get_coordinates(self, p, q, dist):
    # Get coordinates of a point at a distance 'dist' from point 'p',
    # on a line that passes through points 'p' and 'q'.
    distance_ratio = dist / self.distance
     
    x0 = p[0]
    y0 = p[1]
    x1 = q[0]
    y1 = q[1]

    x = ((1 - distance_ratio) * x0) + distance_ratio * x1
    y = ((1 - distance_ratio) * y0) + distance_ratio * y1

    return [x, y]

  
  def print_itinerary(self):
    print("Flight start time: ", self.start_time, " seconds")
    print("Flight finish time: ", self.finish_time, " seconds")
    print("Flight duration: ", self.duration, " seconds")
    print("Flight path (list of coordinates): ", *self.flight_path,sep='\n')



