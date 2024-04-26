import numpy as np
import matplotlib as mlp
from matplotlib import pyplot as plt
import helper
from path_generation.PathGenerator import PathGenerator

figures = 1


desired_title = ""
color_scheme = "viridis"


def visualise_map_by_path(path, graph_name, title):
    global figures
    plt.figure(figures, figsize=(6, 6))
    with open(path, "r", encoding="utf-8-sig") as file:
        lines = file.readlines()
    lines = lines[1:]
    x_points = []
    y_points = []
    sizes = []
    #  sensing points
    for line in lines:
        line = line.strip("\n").split(",")
        x = float(line[2])
        y = float(line[3])
        sensing_value = float(line[5])
        if line[0] == "SENSE":
            x_points.append(x)
            y_points.append(y)
            sizes.append(sensing_value)
    plt.scatter(x_points, y_points, c=sizes, s=100., marker="s", cmap=color_scheme)
    #  base stations
    x_points = []
    y_points = []
    for line in lines:
        line = line.strip("\n").split(",")
        x = float(line[2])
        y = float(line[3])
        sensing_value = float(line[5])
        if line[0] == "BASE":
            x_points.append(x)
            y_points.append(y)
    plt.scatter(x_points, y_points, s=100., marker="^")
    plt.title(title)
    plt.xticks([]), plt.yticks([])
    figures += 1
    plt.savefig(graph_name)


def visualise_sensing():
    global figures
    plt.figure(figures, figsize=(6, 6))
    pg = PathGenerator()
    results = pg.generate_paths(raw=True)
    results = [np.array(result[1]) for result in results]
    final_sensing = sum(results)
    requirements = pg.read_sensing_requirements()
    x_points = []
    y_points = []
    cs = []
    for cell, sensing in zip(requirements["sense"], final_sensing):
        x_points.append(float(cell["x"]))
        y_points.append(float(cell["y"]))
        cs.append(float(sensing))
    plt.scatter(x_points, y_points, c=cs, s=100., marker="s", cmap=color_scheme)
    #  Plot base stations
    x_points = []
    y_points = []
    for base in requirements["base"]:
        x_points.append(float(base["x"]))
        y_points.append(float(base["y"]))
    plt.scatter(x_points, y_points, s=100., marker="^", c="b")
    plt.title(f"Actual Drone Sensing ({desired_title})")
    plt.xticks([]), plt.yticks([])
    figures += 1


if __name__ == "__main__":
    visualise_map_by_path(f"/home/c41/Drones-Testbed/examples/pneuma_grid.csv",
                          f"../results/pneuma_grid.png",
                          f"pNEUMA Sensing Requirements as Grid")
    visualise_map_by_path(f"/home/c41/Drones-Testbed/examples/pneuma_points.csv",
                          f"../results/pneuma_points.png",
                          f"pNEUMA Sensing Requirements as Points")
    plt.show()
    vehicles = ["Car", "Bus", "HeavyVehicle", "MediumVehicle", "Motorcycle", "Taxi"]
    # for vehicle in vehicles:
    #     visualise_map_by_path(f"/home/c41/Drones-Testbed/examples/pneuma_grid_{vehicle}.csv",
    #                           f"../results/pneuma_grid_{vehicle}.png",
    #                           f"pNEUMA Sensing Requirements as Grid ({vehicle})")
    #     visualise_map_by_path(f"/home/c41/Drones-Testbed/examples/pneuma_points_{vehicle}.csv",
    #                           f"../results/pneuma_points_{vehicle}.png",
    #                           f"pNEUMA Sensing Requirements as Points ({vehicle})")
