import numpy as np
from matplotlib import pyplot as plt

from path_generation.PathGenerator import PathGenerator


def sensing_mismatch():
    plt.figure(1, figsize=(8, 6))
    pg = PathGenerator()
    results = pg.generate_paths(raw=True)
    results = [np.array(result[1]) for result in results]
    final_sensing = sum(results)
    final_sensing = final_sensing.tolist()
    requirements = pg.read_sensing_requirements()
    mismatch_vector = []
    for cell_s, cell_r in zip(final_sensing, requirements["sense"]):
        mismatch_vector.append(cell_s - cell_r["value"])
    print(mismatch_vector)
    x_points = []
    y_points = []
    cs = []
    for cell, sensing in zip(requirements["sense"], mismatch_vector):
        x_points.append(float(cell["x"]))
        y_points.append(float(cell["y"]))
        cs.append(float(sensing))
    scats = plt.scatter(x_points, y_points, c=cs, s=100., marker="s", cmap="seismic")
    #  Plot base stations
    x_points = []
    y_points = []
    for base in requirements["base"]:
        x_points.append(float(base["x"]))
        y_points.append(float(base["y"]))
    plt.scatter(x_points, y_points, s=100., marker="^", c="b")
    plt.title(f"Actual Drone Sensing")
    plt.colorbar(scats, orientation="vertical")


if __name__ == '__main__':
    sensing_mismatch()
    plt.show()
