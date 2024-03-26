from .Swarm_Constants import EPOS_TIMESTEP

class Input_Parser:
  # Class to convert EPOS output to path execution input.
  
  def __init__(self, epos_output):
    # Initialise input parser.
    self.epos_output = epos_output
    self.parsed_input = self.parse_input(epos_output)

  def parse_input(self, epos_output):
    # Convert EPOS output to path execution input.
    parsed_input = []
    for drone in epos_output:
        drone_plan = []
        for i in range(len(epos_output[drone])):        
            different_coordinates = True
            if not i == 0:
                different_coordinates = not epos_output[drone][i] == epos_output[drone][i-1]

            if different_coordinates:
                coordinates = list(epos_output[drone][i])
                duration = EPOS_TIMESTEP
                drone_stop = [coordinates, duration]
                drone_plan.append(drone_stop)
                drone_plan[-1][0].pop()
            else:
                drone_plan[-1][1] += EPOS_TIMESTEP
        parsed_input.append(drone_plan)
    print("parsed input: ", parsed_input)
    return parsed_input



