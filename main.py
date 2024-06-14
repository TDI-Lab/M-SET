"""
Workflow executed from here.
"""


import copy
from cdca.src.Input_Parser import Input_Parser
from cdca.src.Potential_Fields_Collision_Avoidance import Potential_Fields_Collision_Avoidance
from cdca.src.Swarm_Control import Swarm_Control
from cdca.src.Basic_Collision_Avoidance import Basic_Collision_Avoidance
from experiments.MeasureSensing import MeasureSensing
from experiments.VisualiseData import VisualiseData

from path_generation.PathGenerator import PathGenerator
import csv
import json
import os
import os.path
import configparser
from random import randint
import matplotlib.pyplot as plt
import numpy as np
class Config:
    def __init__(self, config_file_path='drone_sense.properties'):
        self.config_file_path = config_file_path
        # Check if the config file exists
        if not os.path.isfile(config_file_path):
            print(f"Config file '{config_file_path}' not found.")
            return

        self.config = configparser.ConfigParser()
        self.config.optionxform = str  # preserve case for options

        self.config.read(config_file_path)

def sum_hover_times(plans):
    total_duration = 0
    for drone in plans:
        for plan in drone:
            total_duration += plan[1]
    return total_duration
        
def write_results_to_csv(data, config, experiment_name=None,greedy=False, testbed=False):

    # Now you can access the values in the config file like this:
    mission_name = config.config.get('global', 'MissionName')
    if greedy:
        mission_name += '_greedy'
    n_drones = config.config.get('global', 'NumberOfDrones')

    if testbed:
        mission_name += '_testbed'

    if experiment_name is not None:
        #if folder called experiment_name at 'experiments/results/' does not exist, create it
        if not os.path.exists('experiments/results/'+experiment_name):
            os.makedirs('experiments/results/'+experiment_name)
            experiment_name = experiment_name + '/'
    else:
        experiment_name = ''
    results_path = 'experiments/results/'+experiment_name + '/' + mission_name+'_results.csv'

    header_exists = os.path.isfile(results_path)
    
    # Write the plans and the results to the CSV file
    with open(results_path, 'a', newline='') as f:
        writer = csv.writer(f)
         # Check if the header has already been written

        # Write the header if it doesn't exist
        if not header_exists:
            writer.writerow(['Strategy', 'n_drones', 'Plan', 'plan overflow', 'Sensing Mismatch %','UnderSensing %','Oversensing %','Total Collisions', 'Cross Collisions', 'Parallel Collisions', 'Dest. Occupied Collisions', 'Total Flights Distance', 'Total Collision Distance', 'Risk of Collision', 'Total Duration of Flights', 'Total Hover Duration',  'Number of Flights', 'Average Collisions per Flight'])
        
        # Write the data
        for strategy, _ in data['plans'].items():
            collisions = data['results'][strategy]
            sensing_mismatch, undersensing_percentage,  oversensing_percentage  = data['sensing mismatch'][strategy]
            plans = data['plans'][strategy]
            # Truncate plans to a maximum length of 32700 characters
            plans_str = str(plans)
            plans_truncated = plans_str[:32700]
            plans_overflow = plans_str[32700:]
            if len(plans_overflow) > 32700:
                plans_overflow = "Too long to display"
                
            # Join the plans into a single string
            writer.writerow([strategy, n_drones, plans_truncated, plans_overflow, sensing_mismatch, undersensing_percentage,  oversensing_percentage, collisions['number_of_collisions'], collisions['number_of_cross_collisions'],collisions['number_of_parallel_collisions'], collisions['number_of_dest_occupied_collisions'], collisions['total_flights_distance'], collisions['total_collision_distance'], collisions['risk_of_collision'], collisions['total_duration_of_flights'],collisions['total_hover_duration'], collisions['number_of_flights'], collisions['average_collisions_per_flight']])
        
        # Add a blank line for readability
        writer.writerow([])

def create_new_random_sensing_mission(size_n, size_m):
    mission_name = f"{size_n}x{size_m}_random.csv"
    rows = ["type,id,x,y,z,value\n"]
    #  Create sensing cells
    cell_id = 0
    for i in range(1, size_n + 1):
        for j in range(1, size_m + 1):
            sensing_value = randint(1, 10)
            new_row = f"SENSE,{cell_id},{i},{j},1,{sensing_value}\n"
            rows.append(new_row)
            cell_id += 1


    rows.append(f"BASE,1,0,0,0,0\n")
    rows.append(f"BASE,2,{size_n + 1},0,0,0\n")
    rows.append(f"BASE,3,0,{size_m + 1},0,0\n")
    rows.append(f"BASE,4,{size_n + 1},{size_m + 1},0,0\n")

    rows.append(f"BASE,5,{(size_n + 1) / 2},0,0,0\n")
    rows.append(f"BASE,6,0,{(size_m + 1) / 2},0,0\n")
    rows.append(f"BASE,7,{size_n + 1},{(size_m + 1) / 2},0,0\n")
    rows.append(f"BASE,8,{(size_n + 1) / 2},{(size_m + 1) },0,0\n")

    rows.append(f"BASE,9,{(size_n + 1) * 0.75},0,0,0\n")
    rows.append(f"BASE,10,0,{(size_m + 1) * 0.75},0,0\n")
    rows.append(f"BASE,11,{size_n + 1},{(size_m + 1)* 0.75},0,0\n")
    rows.append(f"BASE,12,{(size_n + 1) * 0.75},{(size_m + 1) },0,0\n")

    rows.append(f"BASE,13,{(size_n + 1) * 0.25},0,0,0\n")
    rows.append(f"BASE,14,0,{(size_m + 1) * 0.25},0,0\n")
    rows.append(f"BASE,15,{size_n + 1},{(size_m + 1)* 0.25},0,0\n")
    rows.append(f"BASE,16,{(size_n + 1) * 0.25},{(size_m + 1) },0,0\n")
    
    with open(f"examples/{mission_name}", "w") as file:
        for row in rows:
            file.write(row)

    return mission_name


def interpolate_points(x1, y1, x2, y2, n_points):
    x_values = np.linspace(x1, x2, n_points)
    y_values = np.linspace(y1, y2, n_points)
    return list(zip(x_values, y_values))
def add_base_stations_to_pneuma_sensing_mission(n_base_stations, mission_name):
    rows = ["type,id,x,y,z,value\n"]
   
   
    corner_stations = [[-30,-30],[1631.4428430001244,-30],[1631.4428430001244,1919.4018143002982],[-30,1919.4018143002982]]

    x_values = []
    y_values = []

    for i in range(len(corner_stations)):
        x1, y1 = corner_stations[i]
        x2, y2 = corner_stations[(i+1) % len(corner_stations)]
        points = interpolate_points(x1, y1, x2, y2, n_base_stations // 4)
        for point in points:
            x_values.append(point[0])
            y_values.append(point[1])
        
    # Shuffle the points but keep every x's corresponding y at the new position
    indices = np.arange(len(x_values))
    np.random.shuffle(indices)
    x_values = np.array(x_values)[indices]
    y_values = np.array(y_values)[indices]


    with open(f"examples/{mission_name}", "a") as file:
        for i in range(len(x_values)):
            row = f"BASE,{i},{x_values[i]},{y_values[i]},0,0\n"
            file.write(row)

    # Plotting the x, y points
    plt.scatter(x_values, y_values, s=10)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Base Stations')
    plt.show()

    return mission_name

def create_new_testbed_sensing_mission():
    mission_name = f"2x3_random_testbed.csv"
    rows = ["type,id,x,y,z,value\n"]
    #  Create sensing cells
   
    rows.append(f"SENSE,0,-0.5533,-0.235,1,{randint(1, 10)}\n")
    rows.append(f"SENSE,1,0,-0.235,1,{randint(1, 10)}\n")
    rows.append(f"SENSE,2.0,5533,-0.235,1,{randint(1, 10)}\n")
    rows.append(f"SENSE,3,-0.5533,0.235,1,{randint(1, 10)}\n")
    rows.append(f"SENSE,4,0,0.235,1,{randint(1, 10)}\n")
    rows.append(f"SENSE,5,0.5533,0.235,1,{randint(1, 10)}\n")

    
    rows.append(f"BASE,0,-0.8299,-0.47,0,0\n")
    rows.append(f"BASE,1,0.8299,-0.47,0,0\n")
    rows.append(f"BASE,2,0.8299,0.47,0,0\n")
    rows.append(f"BASE,3,-0.8299,0.47,0,0\n")


    
    with open(f"examples/{mission_name}", "w") as file:
        for row in rows:
            file.write(row)

    return mission_name

def experiment_iteration(n_drones, mission_name):

    pg = PathGenerator()
    plans = pg.generate_paths()

    for plant, path in plans.items():
        print(f"plan: {plant}, path: {path}")

    # if n_drones == 1:
    #     plans = [plans]

    input_p = Input_Parser(plans)
    parsed_plans = input_p.parsed_input
    
    print("")
    for planx in parsed_plans:
        print(planx)
    print("")

    sensing = MeasureSensing(f"examples/{mission_name}")
    # swarm_controller4 = Swarm_Control(copy.deepcopy(parsed_plans), Dependency_Collision_Avoidance(visualise=True))
    # swarm_controller4.detect_potential_collisions()
    swarm_controller = Swarm_Control(copy.deepcopy(parsed_plans), Basic_Collision_Avoidance())
    swarm_controller2 = Swarm_Control(copy.deepcopy(parsed_plans), Potential_Fields_Collision_Avoidance(visualise=False))
    swarm_controller3 = Swarm_Control(copy.deepcopy(parsed_plans), Basic_Collision_Avoidance())
    # swarm_controller.visualise_swarm()

    print("Starting PF CA")
    swarm_controller2.detect_potential_collisions()
    print("Starting Basic CA")

    swarm_controller3.detect_potential_collisions()
    # swarm_controller.visualise_swarm()

    # swarm_controller2.visualise_swarm()



    no_ca_plans =  swarm_controller.plans
    pf_plans = swarm_controller2.plans
    basic_plans = swarm_controller3.plans



    return {
    'plans': {
        'no_ca':no_ca_plans,
        'pf_ca': pf_plans,
        'basic_ca':basic_plans,
    },
    'results': {
        'no_ca': swarm_controller.get_offline_collision_stats(),
        'pf_ca': swarm_controller2.get_offline_collision_stats(),
        'basic_ca': swarm_controller3.get_offline_collision_stats(),
    },
    'sensing mismatch': {
        'no_ca': sensing.measure_sensing(no_ca_plans),
        'pf_ca': sensing.measure_sensing(pf_plans),
        'basic_ca': sensing.measure_sensing(basic_plans),
    }
}
def experiment_testbed():
    config = Config('drone_sense.properties')

    n_iterations = 200
    drones = [1,2,3,4]
  

    abs_path = os.path.abspath('.')
    config.config.set('global', 'MissionName', f"2x3_random_testbed")

    config.config.set('global', 'MissionFile', f"{abs_path}/examples/2x3_random_testbed.csv")
    
    config.config.set('drone', 'BatteryCapacity', f"2700")
    config.config.set('drone', 'BodyMass', f"0.027")
    config.config.set('drone', 'BatteryMass', f"0.005")
    config.config.set('drone', 'PowerEfficiency', f"1.25")
    for _ in range(n_iterations):
        for n_drones in drones:
            config.config.set('global', 'NumberOfDrones', f"{n_drones}")

            with open(config.config_file_path, 'w') as configfile:
                config.config.write(configfile)
            mission_name = create_new_testbed_sensing_mission()

            data = experiment_iteration(n_drones, mission_name)
            write_results_to_csv(data, config,experiment_name='test_log_repulsion_200iterations', testbed=True)
                
def run_experiment_1and2(greedy):
     # a list of n m for each experiment grid
    experiment_sizes = [[2,3],[3,3],[4,4],[5,5],[6,6],[8,8],[10,10],[12,12]]

    config = Config('drone_sense.properties')

    n_iterations = 200
    drones = [5,6,8,10,12,16]
    # # drones = [5,6,7,8]
    # config.config.set('drone', 'BatteryCapacity', f"2700")
    # config.config.set('drone', 'BodyMass', f"0.027")
    # config.config.set('drone', 'BatteryMass', f"0.005")
    # config.config.set('drone', 'PowerEfficiency', f"1.25")
    for i in range(len(experiment_sizes)):
        abs_path = os.path.abspath('.')
        config.config.set('global', 'MissionName', f"random_{experiment_sizes[i][0]}x{experiment_sizes[i][1]}")

        config.config.set('global', 'MissionFile', f"{abs_path}/examples/{experiment_sizes[i][0]}x{experiment_sizes[i][1]}_random.csv")
        

        for _ in range(n_iterations):
            for n_drones in drones:
                config.config.set('global', 'NumberOfDrones', f"{n_drones}")

                with open(config.config_file_path, 'w') as configfile:
                    config.config.write(configfile)
                mission_name = create_new_random_sensing_mission(experiment_sizes[i][0], experiment_sizes[i][1])

                data = experiment_iteration(n_drones, mission_name)
                write_results_to_csv(data, config, experiment_name='test',greedy=greedy)

def run_pneuma_experiment():

    config = Config('drone_sense.properties')
    abs_path = os.path.abspath('.')

    missions = ['pneuma_points', 'pneuma_grid']

    #set drone properties
    config.config.set('drone', 'BatteryCapacity', f"275000")
    config.config.set('drone', 'BodyMass', f"1.07")
    config.config.set('drone', 'BatteryMass', f"0.31")
    config.config.set('drone', 'PowerEfficiency', f"0.8")
    
    for mission in missions:
        config.config.set('global', 'MissionName', mission)

        config.config.set('global', 'MissionFile', f"{abs_path}/examples/{mission}.csv")
        
        
        n_iterations = 1
        drones = [4]#,10,32,64,128,256,512]


        for _ in range(n_iterations):
            for n_drones in drones:
                config.config.set('global', 'NumberOfDrones', f"{n_drones}")

                with open(config.config_file_path, 'w') as configfile:
                    config.config.write(configfile)

                data = experiment_iteration(n_drones, mission+".csv")
                write_results_to_csv(data, config, experiment_name='pneuma',greedy=False)
            




if __name__ == '__main__':
    # add_base_stations_to_pneuma_sensing_mission(512, 'pneuma_points.csv')
    # experiment_testbed()
    # run_pneuma_experiment()
    # # run_pneuma_experiment()
    # # run_experiment_1and2(greedy=False)
    # # experiment_testbed()
    # # experiment_testbed()
    # # data_paths = ['experiments/results/random_2x3_results.csv','experiments/results/random_3x3_results.csv','experiments/results/random_4x4_results.csv','experiments/results/random_5x5_results.csv', 
    # #               'experiments/results/random_6x6_results.csv',  'experiments/results/random_8x8_results.csv',
    # #               'experiments/results/random_10x10_results.csv',  'experiments/results/random_12x12_results.csv']
    
    testbed_data_path = ['experiments/results/pnuema/TESTBED_0.1EPOS_5s_pneuma_real_data_4_drone/priority.csv']
    testbed_data_path2 = ['experiments/results/pnuema/TESTBED_NOPRIORITY_0.1EPOS_5s_pneuma_real_data_4_drone/new_pneuma_points_results.csv']

    # new_paths = []
    # for path in testbed_data_path:
    #     #append replace _results to _greedy_results
    #     new_path = path.replace('/results/', '/results/test_log_repulsion_200iterations/')
    #     new_paths.append(new_path)

    # Visualise the data
    # for path in new_paths:
    #     print(path)
    #     # extract the 5x5 from the path
    #     map_name = path.split('/')[-1].split('_')[1] + "Testbed"
    #     print(map_name)
    #     vd = VisualiseData(path, 'experiments/results/')
    #     vd.plotNumAgentsVsSensingAccuracy(map_name)
    # data_path = [ 'experiments/results/random_2x3_results.csv']
    map_name = "Priority Testbed"
    vd = VisualiseData(testbed_data_path, 'experiments/results/')
    vd.plotPowerEstimation(map_name)
    vd.plottypesOfCollisions(map_name)
    vd.plotNumAgentsVsRiskOfCollision(map_name)
    vd.plotAgents_vs_TotalDuration(map_name)
    vd.plotNumAgentsVsSensingAccuracy(map_name)
    vd.plotNumAgentsVsCollisions(map_name)

    map_name = "No Priority Testbed"
    vd = VisualiseData(testbed_data_path2, 'experiments/results/')
    vd.plotPowerEstimation(map_name)
    vd.plottypesOfCollisions(map_name)
    vd.plotNumAgentsVsRiskOfCollision(map_name)
    vd.plotAgents_vs_TotalDuration(map_name)
    vd.plotNumAgentsVsSensingAccuracy(map_name)
    vd.plotNumAgentsVsCollisions(map_name)
   