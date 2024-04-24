import math
from copy import deepcopy
from statistics import mean

import numpy as np
from matplotlib import pyplot as plt
from numpy import std

from path_generation.PathGenerator import PathGenerator
from helper import STANDARD_SYSTEM_CONF, write_results_to_file
from path_generation.ConfigManager import ConfigManager


MAX_DRONES_EXP = 6
REPEATS_PER_EXPERIMENT = 5


def sensing_mismatch_grid_experiment(file_suffix="", greedy=False):
    if file_suffix != "":
        file_suffix = "_" + file_suffix
    cm = ConfigManager()
    cm.set_target_path("/home/c41/Drones-Testbed/drone_sense.properties")
    efficiencies = []
    for i in range(1, MAX_DRONES_EXP + 1):
        cur_eff = []
        for repeat in range(0, REPEATS_PER_EXPERIMENT):
            props = deepcopy(STANDARD_SYSTEM_CONF)
            props["global"]["MissionName"] = f"pneuma_grid{file_suffix}"
            props["global"]["MissionFile"] = f"/home/c41/Drones-Testbed/examples/pneuma_grid{file_suffix}.csv"
            props["global"]["NumberOfDrones"] = 2**i
            props["path_generation"]["PathMode"] = "greedy" if greedy else "normal"
            props["path_generation"]["NumberOfPlans"] = 32
            cm.write_config_file(props)
            cur_eff.append(mean(sensing_mismatch(plot=False)))
        efficiencies.append({"mean": mean(cur_eff), "std": std(cur_eff)})
    return efficiencies


def sensing_mismatch_points_experiment(file_suffix="", greedy=False):
    if file_suffix != "":
        file_suffix = "_" + file_suffix
    cm = ConfigManager()
    cm.set_target_path("/home/c41/Drones-Testbed/drone_sense.properties")
    efficiencies = []
    for i in range(1, MAX_DRONES_EXP+1):
        cur_eff = []
        for repeat in range(0, REPEATS_PER_EXPERIMENT):
            props = deepcopy(STANDARD_SYSTEM_CONF)
            props["global"]["MissionName"] = f"pneuma_points{file_suffix}"
            props["global"]["MissionFile"] = f"/home/c41/Drones-Testbed/examples/pneuma_points{file_suffix}.csv"
            props["global"]["NumberOfDrones"] = 2**i
            props["path_generation"]["PathMode"] = "greedy" if greedy else "normal"
            props["path_generation"]["NumberOfPlans"] = 32
            cm.write_config_file(props)
            cur_eff.append(mean(sensing_mismatch(plot=False)))
        efficiencies.append({"mean": mean(cur_eff), "std": std(cur_eff)})
    return efficiencies


def sensing_mismatch(plot=True):
    plt.figure(1, figsize=(8, 6))
    pg = PathGenerator()
    results = pg.generate_paths(raw=True)
    results = [np.array(result[1]) for result in results]
    final_sensing = sum(results)
    final_sensing = final_sensing.tolist()
    requirements = pg.read_sensing_requirements()
    accuracy_vector = []
    efficiency_vector = []
    for cell_s, cell_r in zip(final_sensing, requirements["sense"]):
        # accuracy_vector.append(log10(1/((cell_s - cell_r["value"]) ** 2)))
        efficiency_vector.append((cell_s/cell_r["value"])*100)
    if plot:
        x_points = []
        y_points = []
        cs = []
        for cell, sensing in zip(requirements["sense"], efficiency_vector):
            x_points.append(float(cell["x"]))
            y_points.append(float(cell["y"]))
            cs.append(float(sensing))
        scats = plt.scatter(x_points, y_points, c=cs, s=100., marker="s", cmap="bone")
        #  Plot base stations
        x_points = []
        y_points = []
        for base in requirements["base"]:
            x_points.append(float(base["x"]))
            y_points.append(float(base["y"]))
        plt.scatter(x_points, y_points, s=100., marker="^", c="b")
        plt.title(f"Actual Drone Sensing")
        plt.colorbar(scats, orientation="vertical")
    return efficiency_vector


def by_vehicle(greedy=False):
    vehicles = ["Car"]#, "Bus", "HeavyVehicle", "MediumVehicle", "Motorcycle", "Taxi"]
    for vehicle in vehicles:
        points_efficiencies = sensing_mismatch_points_experiment(file_suffix=f"{vehicle}", greedy=greedy)
        grid_efficiencies = sensing_mismatch_grid_experiment(file_suffix=f"{vehicle}", greedy=greedy)
        compressed_results = []
        for i in range(1, MAX_DRONES_EXP+1):
            result = [2**i, points_efficiencies[i-1], grid_efficiencies[i-1]]
            result = list(map(str, result))
            compressed_results.append(result)
        write_results_to_file("/home/c41/Drones-Testbed/path_generation/results",
                              f"pneuma_efficiency_{vehicle}_greedy" if greedy else f"pneuma_efficiency_{vehicle}",
                              ["drones", "points", "grid"],
                              compressed_results)


def all_points(greedy=False):
    points_efficiencies = sensing_mismatch_points_experiment(greedy=greedy)
    grid_efficiencies = sensing_mismatch_grid_experiment(greedy=greedy)
    print(points_efficiencies)
    print(grid_efficiencies)
    compressed_results = []
    for i in range(1, MAX_DRONES_EXP + 1):
        result = [2 ** i, points_efficiencies[i - 1]["mean"], points_efficiencies[i - 1]["std"],
                  grid_efficiencies[i - 1]["mean"], grid_efficiencies[i - 1]["std"]]
        result = list(map(str, result))
        compressed_results.append(result)
    write_results_to_file("/home/c41/Drones-Testbed/path_generation/results",
                          f"pneuma_efficiency_greedy" if greedy else f"pneuma_efficiency",
                          ["drones", "points_mean", "points_std", "grid_mean", "grid_std"],
                          compressed_results)


def visualise_efficiency_results(path, save_path):
    with open(path, "r") as file:
        lines = file.readlines()[1:]
    lines = [line.strip("\n").split(",") for line in lines]
    drone_exp = [i for i in range(1, len(lines)+1)]
    points_mean = np.array([float(line[1]) for line in lines])
    points_std = np.array([float(line[2]) for line in lines])
    grid_mean = np.array([float(line[3]) for line in lines])
    grid_std = np.array([float(line[4]) for line in lines])
    fig, ax = plt.subplots()
    ax.plot(drone_exp, points_mean, label="Point Map")
    ax.plot(drone_exp, grid_mean, label="Grid Map")
    ax.fill_between(drone_exp, points_mean - points_std, points_mean + points_std, alpha=0.2)
    ax.fill_between(drone_exp, grid_mean - grid_std, grid_mean + grid_std, alpha=0.2)
    ax.set_xticks(drone_exp)
    ax.legend()
    ax.grid()
    ax.set_title("Mission Efficiency")
    ax.set_xlabel("# Drones ($2^x$)")
    ax.set_ylabel("Mission Efficiency (%)")
    plt.savefig(save_path)


if __name__ == '__main__':
    # by_vehicle(greedy=False)
    # all_points(greedy=True)
    visualise_efficiency_results(
        "/home/c41/Drones-Testbed/path_generation/results/pneuma_efficiency_greedy_26925436179827.csv",
        "../results/pneuma_effiency_greedy_less.png")
    plt.show()
