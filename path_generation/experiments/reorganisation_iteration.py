import numpy as np
from matplotlib import pyplot as plt


def result_reorganisation_iteration(self):
    results_data = self.__retrieve_file_data("num-reorganizations")
    results_data = results_data
    iterations = np.array([int(float(i.split(",")[0])) for i in results_data])
    reorganisations = []
    for line in results_data:
        formatted = [int(float(i)) for i in line.strip("\n").split(",")]
        mean = sum(formatted[1:]) / (len(formatted[1:]))
        reorganisations.append(mean)
    reorganisations = np.array(reorganisations)

    fig, ax = plt.subplots()
    ax.plot(iterations, reorganisations)
    ax.grid()
    ax.set_xlabel("Iterations")
    ax.set_ylabel("Reorganisations")
    ax.set_title("Total Reorganisations during Execution")
    fig.savefig(f"{self.parent_path}/results/reorganization_iteration.png")