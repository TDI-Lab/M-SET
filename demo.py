
import os
from cdca.src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
from cdca.src.Input_Parser import Input_Parser
from cdca.src.Potential_Fields_Collision_Avoidance import Potential_Fields_Collision_Avoidance
from cdca.src.Swarm_Control import Swarm_Control
from path_generation.PathGenerator import PathGenerator
from experiments.MeasureSensing import MeasureSensing


from main import Config


def setup():
    abs_path = os.path.abspath('.')

    #set drone properties
    config = Config('drone_sense.properties')
    config.config.set('drone', 'BatteryCapacity', f"2700")
    config.config.set('drone', 'BodyMass', f"0.027")
    config.config.set('drone', 'BatteryMass', f"0.005")
    config.config.set('drone', 'PowerEfficiency', f"1.25")
    
    config.config.set('global', 'MissionName', 'Demo')
    config.config.set('global', 'MissionFile', f"{abs_path}/examples/testbed.csv")

    with open(config.config_file_path, 'w') as configfile:
                    config.config.write(configfile)

def execute_mission():

    # Generate paths from mission file and drone_sense.properties config file
    pg = PathGenerator()
    plans = pg.generate_paths()

    # Parse the generated paths to input for the collision avoidance algorithms
    ip = Input_Parser(plans)
    parsed_plans = ip.parsed_input

    # Chose the collision avoidance strategy
    collision_strategy1 = Potential_Fields_Collision_Avoidance(visualise=True) # Optionally set to True to visualise the potential fields for each drone
    collision_strategy2 = Basic_Collision_Avoidance()

    # Create a swarm controller with the chosen collision avoidance strategy
    sc1 = Swarm_Control(parsed_plans, collision_strategy1)

    # Visualise the drone plans before collision avoidance
    sc1.visualise_swarm(title="Before Collision Avoidance")

    # Get collision data after collision avoidance
    No_CA_collision_data = sc1.get_offline_collision_stats()

    old_paths = sc1.plans #Saving the old paths for comparison

    # Get sensing data before collision avoidance
    sensing = MeasureSensing(f"examples/testbed.csv")
    sensing_before_CA = sensing.measure_sensing(sc1.plans)

    # Detect and fix potential collisions between drones
    sc1.detect_potential_collisions()
    new_paths = sc1.plans
    # Visualise the drone plans after collision avoidance
    sc1.visualise_swarm(title="After Collision Avoidance")

    # Get collision data after collision avoidance
    CA_collision_data = sc1.get_offline_collision_stats()

    # Get sensing data after collision avoidance
    sensing_after_CA = sensing.measure_sensing(sc1.plans)

    # Optionally, execute the paths on the real drones using the path executor component
    # TODO: Implement the path executor component

    
    # Print the paths
    # print("\nOld Paths: \n")
    # for path in old_paths:
    #     print(path)
    # print("\nNew Paths: \n")
    # for path in new_paths:
    #     print(path)
    


if __name__ == '__main__':
    setup()
    execute_mission()
    




