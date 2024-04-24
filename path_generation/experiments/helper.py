from itertools import groupby
from os import listdir
from pathlib import Path
from statistics import mean
from time import perf_counter

import numpy as np
from numpy import std

from path_generation.PathGenerator import PathGenerator

PLAN_GENERATION_STANDARD_PROPERTIES = {
    "plan": {
        "dataset": "testbed",
        "planNum": 32,
        "agentsNum": 4,
        "timeSlots": 24,
        "maxVisitedCells": 6
    },
    "map": {
        "stationsNum": 4,
        "height": 1,
        "mapLength": 4
    },
    "power": {
        "batteryCapacity": 2700.,
        "bodyMass": 0.027,
        "batteryMass": 0.05,
        "rotorNum": 4,
        "rotorDia": 0.03,
        "projectedBody": 0.0599,
        "projectedBattery": 0.0037,
        "powerEfficiency": 0.8,
        "groundSpeed": 6.94,
        "airDensity": 1.225,
        "airSpeed": 8.5
    }
}

EPOS_STANDARD_PROPERTIES = {
    "dataset": "testbed",
    "numSimulations": 1,
    "numIterations": 32,
    "numAgents": 4,
    "numPlans": 128,
    "numChildren": 2,
    "planDim": 6,
    "shuffle": 0,
    "shuffle_file": "permutation.csv",
    "numberOfWeights": 2,
    "weightsString": "0.0,0.0",
    "behaviours": "same",
    "agentsBehaviourPath": "default",
    "constraint": "SOFT",
    "constraintPlansPath": "default",
    "constraintCostsPath": "default",
    "strategy": "never",
    "periodically.reorganizationPeriod": 3,
    "convergence.memorizationOffset": 5,
    "globalCost.reductionThreshold": 0.5,
    "strategy.reorganizationSeed": 0,
    "globalSignalPath": "",
    "globalCostFunction": "VAR",
    "scaling": "STD",
    "localCostFunction": "INDEX",
    "logger.GlobalCostLogger": "true",
    "logger.LocalCostMultiObjectiveLogger": "true",
    "logger.TerminationLogger": "true",
    "logger.SelectedPlanLogger": "true",
    "logger.GlobalResponseVectorLogger": "true",
    "logger.PlanFrequencyLogger": "true",
    "logger.UnfairnessLogger": "true",
    "logger.GlobalComplexCostLogger": "false",
    "logger.WeightsLogger": "false",
    "logger.ReorganizationLogger": "true",
    "logger.VisualizerLogger": "false",
    "logger.PositionLogger": "true",
    "logger.HardConstraintLogger": "false"
}

STANDARD_SYSTEM_CONF = {
    "global": {
        "MissionName": "testbed",
        "MissionFile": "testbed.csv",
        "NumberOfDrones": 4
    },

    "path_generation": {
        "NumberOfPlans": 8,
        "PathMode": "normal"
    },

    "epos": {
        "EPOSstdout": False,
        "EPOSstderr": False,
        "NumberOfSimulations": 1,
        "IterationsPerSimulation": 32,
        "NumberOfChildren": 2,
        "Shuffle": 0,
        "ShuffleFile": "permuation.csv",
        "NumberOfWeights": 2,
        "WeightsString": "0.0,0.0",
        "behaviours": "same",
        "agentsBehaviourPath": "default",
        "constraint": "SOFT",
        "constraintPlansPath": "default",
        "constraintCostsPath": "default",
        "strategy": "never",
        "periodically.reorganizationPeriod": 3,
        "convergence.memorizationOffset": 5,
        "globalCost.reductionThreshold": 0.5,
        "strategy.reorganizationSeed": 0,
        "globalSignalPath": "",
        "globalCostFunction": "MIS",
        "scaling": "STD",
        "localCostFunction": "INDEX",
    },

    "drone": {
        "BatteryCapacity": 275000,
        "BodyMass": 1.07,
        "BatteryMass": 0.31,
        "NumberOfRotors": 4,
        "RotorDiameter": 0.35,
        "ProjectedBodyArea": 0.0599,
        "ProjectedBatteryArea": 0.0037,
        "PowerEfficiency": 0.8,
        "GroundSpeed": 6.94,
        "AirSpeed": 8.5
    },

    "environment": {
        "AirDensity": 1.225
    }
}


def get_parent_path():
    return Path(__file__).parent.resolve()


def __compute_mean_several_runs(self, data):
    avg_costs = []
    for line in data:
        formatted = [abs(float(i)) for i in line.strip("\n").split(",")[3:]]
        formatted = list(filter(lambda x: x != float("inf") and x != float("-inf"), formatted))
        if len(formatted) == 0:
            mean = 0.
        else:
            mean = sum(formatted) / len(formatted)
        avg_costs.append(mean)
    return avg_costs


def __compute_std_several_runs(self, data):
    avg_std = []
    for line in data:
        formatted = [abs(float(i)) for i in line.strip("\n").split(",")[3:]]
        formatted = list(filter(lambda x: x != float("inf") and x != float("-inf"), formatted))
        avg_std.append(std(formatted))
    return avg_std


def __load_target(self, target):
    # 3. Load target
    goal = None
    with open(target, mode="r", encoding="utf-8-sig") as file:
        goal = file.readlines()
    goal = np.array([float(i) for i in goal[0].strip("\n").split(",")])
    return goal


def __retrieve_file_data(self, filename, headers=False):
    output_dir = f"{self.parent_path}/EPOS/output"
    result_dir = listdir(output_dir)[0]
    results_file = f"{output_dir}/{result_dir}/{filename}.csv"
    with open(results_file) as file:
        lines = file.readlines()
    if headers:
        return lines
    return lines[1:]


def __generate_collision_probabilities(self):
    #  Run path generation
    pg = PathGenerator()
    pg.generate_paths(True)
    #  Generate combinations and their first respective runs
    results = self.__retrieve_file_data("agents-position")
    total_combinations = set([tuple(map(int, i.strip("\n").split(",")[:-1])) for i in results])
    #  Separate all runs
    runs = self.__retrieve_file_data("selected-plans")
    all_runs = [line.strip("\n").split(",") for line in runs]
    all_runs = [list(map(int, line)) for line in all_runs]
    all_runs = [list(group) for k, group in groupby(all_runs, lambda x: x[0])]
    runs_for_combinations = {}
    for combination in total_combinations:
        if combination[1:] not in runs_for_combinations:
            runs_for_combinations[combination[1:]] = [all_runs[combination[0] - 1][-1][2:]]
        else:
            runs_for_combinations[combination[1:]].append(all_runs[combination[0] - 1][-1][2:])
    for combination, indexes in runs_for_combinations.items():
        runs_for_combinations[combination] = []
        for index_set in indexes:
            plans = pg.convert_data_to_table(pg.generation_manager.extract_results(index_set))
            runs_for_combinations[combination].append(plans)
    #  Calculate collisions for each combination
    combination_collisions = {}
    for combination, plans in runs_for_combinations.items():
        collisions = 0
        moves = 0
        combination_collisions[combination] = 0
        for plan in plans:
            agent_movements = [i[1] for i in list(plan.items())]
            for agent1_plan in agent_movements:
                for agent2_plan in agent_movements:
                    if agent1_plan == agent2_plan:
                        continue
                    for i in range(1, min(len(agent1_plan), len(agent2_plan))):
                        moves += 1
                        if self.check_collision(agent1_plan[i - 1], agent2_plan[i - 1], agent1_plan[i],
                                                agent2_plan[i]):
                            collisions += 1
        combination_collisions[combination] = (collisions, moves)
    #  Calculate the probability for a collision from each combination
    cur_cps = []
    for collisions, moves in combination_collisions.values():
        cur_cps.append((float(collisions) / moves) * 100.)
    return mean(cur_cps), std(cur_cps)


def check_collision(self, p1, p2, new_p1, new_p2, bounds=(-1, 1, -1, 1)):
    #  bounds=(x0, x1, y0, y1)
    a, b = 0., 0.
    if new_p1[0] - p1[0] != 0.:
        a = (new_p1[1] - p1[1]) / (new_p1[0] - p1[0])
    if new_p2[0] - p2[0] != 0.:
        b = (new_p2[1] - p2[1]) / (new_p2[0] - p2[0])
    if a == b:
        return False
    #  Compute line intercepts for each trajectory
    c = p1[1] - a * p1[0]
    d = p2[1] - b * p2[0]
    #  Compute intersection
    p_x = (d - c) / (a - b)
    p_y = a * p_x + c
    #  Check if they collide within the grid bounds
    if not (bounds[0] <= p_x <= bounds[1] and bounds[2] <= p_y <= bounds[3]):
        return False
    return True


def __gaussian_filter(self, kernel_size, sigma=1, muu=0):
    #  SOURCE: https://www.projectpro.io/recipes/generate-generic-2d-gaussian-like-array
    # Initializing value of x,y as grid of kernel size
    # in the range of kernel size
    x, y = np.meshgrid(np.linspace(-1, 1, kernel_size),
                       np.linspace(-1, 1, kernel_size))
    dst = np.sqrt(x ** 2 + y ** 2)
    # lower normal part of gaussian
    g = np.exp(-((dst - muu) ** 2 / (2.0 * sigma ** 2)))
    return g


def write_results_to_file(result_dir, result_name, headers, results):
    stamp = str(perf_counter()).replace(".", "")
    rows = [",".join(headers) + "\n"]
    for result in results:
        rows.append(",".join(result) + "\n")
    with open(f"{result_dir}/{result_name}_{stamp}.csv", "w") as file:
        file.writelines(rows)
    print(f"Results saved under {result_dir}/{result_name}_{stamp}.csv")
