"""
Will eventually move these into a results factory when it's available.
"""
from os import listdir
from pathlib import Path
from pprint import pprint

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

from path_generation.PathGenerator import PathGenerator


class PathGenerationResultsCollector:

    def __init__(self):
        self.parent_path = Path(__file__).parent.resolve()
        self.pg = PathGenerator()

    def collect_results(self):
        funcs = list(filter(lambda x: x[0:6] == "result", dir(PathGenerationResultsCollector)))
        for func in funcs:
            result_func = getattr(PathGenerationResultsCollector, func)
            result_func(self)

    def perform_experiments(self):
        funcs = list(filter(lambda x: x[0:10] == "experiment", dir(PathGenerationResultsCollector)))
        for func in funcs:
            result_func = getattr(PathGenerationResultsCollector, func)
            result_func(self)

    def __retrieve_file_data(self, filename):
        output_dir = f"{self.parent_path}/EPOS/output"
        result_dir = listdir(output_dir)[0]
        results_file = f"{output_dir}/{result_dir}/{filename}.csv"
        with open(results_file) as file:
            lines = file.readlines()
        return lines

    def result_reorganisation_iteration(self):
        results_data = self.__retrieve_file_data("num-reorganizations")
        headers = results_data[0]
        results_data = results_data[1:]
        iterations = np.array([int(float(i.split(",")[0])) for i in results_data])
        reorganisations = []
        for line in results_data:
            formatted = [int(float(i)) for i in line.strip("\n").split(",")]
            mean = sum(formatted[1:])/(len(formatted[1:]))
            reorganisations.append(mean)
        reorganisations = np.array(reorganisations)
        # x = np.linspace(iterations.min(), iterations.max(), 1000)
        # x_y_spline = make_interp_spline(iterations, reorganisations)
        # y = x_y_spline(x)

        fig, ax = plt.subplots()
        ax.plot(iterations, reorganisations)
        ax.grid()
        ax.set_xlabel("Iterations")
        ax.set_ylabel("Reorganisations")
        ax.set_title("Total Reorganisations during Execution")
        fig.savefig(f"{self.parent_path}/results/reorganization_iteration.png")

    def result_iteration_cost(self):
        results_data = self.__retrieve_file_data("global-cost")
        headers = results_data[0]
        results = results_data[1:]
        iterations = np.array([int(float(i.split(",")[0])) for i in results])
        avg_costs = []
        for line in results:
            formatted = [float(i) for i in line.strip("\n").split(",")[3:]]
            formatted = list(filter(lambda x: x != float("inf") and x != float("-inf"), formatted))
            mean = sum(formatted[1:]) / (len(formatted[1:]))
            avg_costs.append(mean)

        fig, ax = plt.subplots()
        ax.plot(iterations, avg_costs)
        ax.grid()
        ax.set_xlabel("Iterations")
        ax.set_ylabel("Average Global Cost")
        ax.set_title("Global Cost Reduction")
        fig.savefig(f"{self.parent_path}/results/iteration_cost.png")

    def experiment_reorganisation_mismatch(self):
        pass

    def experiment_average_mismatch(self):
        results = []
        mismatches = []
        for _ in range(0, 10):
            results.append(self.pg.generate_paths())
        #  Pre-defined sensing target, change to dynamic
        target = np.array([5., 9., 4., 5., 4., 8.])
        avg_sensing = np.zeros((1, 6))
        for result in results:
            total_sensing = np.zeros((1, 6))
            for plan in result:
                total_sensing += np.array(plan[1])
            avg_sensing += total_sensing
            mismatches.append(total_sensing - target)
        avg_sensing /= len(results)
        avg_sensing = avg_sensing[0]
        avg_mismatch = np.zeros((1, 6))
        for mismatch in mismatches:
            avg_mismatch += abs(mismatch)
        avg_mismatch /= len(mismatches)
        avg_mismatch = avg_mismatch[0]
        print(avg_sensing)
        print(avg_mismatch)


if __name__ == '__main__':
    pgr = PathGenerationResultsCollector()
    pgr.perform_experiments()
