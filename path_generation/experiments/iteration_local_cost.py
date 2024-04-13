import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import make_interp_spline


def result_iteration_local_cost(self):
    results_data = self.__retrieve_file_data("local-cost")
    headers = results_data[0]
    results = results_data[1:]
    iterations = np.array([int(float(i.split(",")[0])) for i in results])
    avg_costs = []
    for line in results:
        formatted = [float(i) for i in line.strip("\n").split(",")[3:]]
        formatted = list(filter(lambda x: x != float("inf") and x != float("-inf"), formatted))
        mean = sum(formatted) / (len(formatted))
        avg_costs.append(mean)

    x = np.linspace(iterations.min(), iterations.max(), 1000)
    x_y_spline = make_interp_spline(iterations, avg_costs)
    y = x_y_spline(x)

    fig, ax = plt.subplots()
    ax.plot(x, y)
    ax.grid()
    ax.set_xlabel("Iterations")
    ax.set_ylabel("Average Local Cost")
    ax.set_title("Local Cost Reduction")
    fig.savefig(f"{self.parent_path}/results/iteration_local_cost.png")