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

    def result_iteration_global_cost(self):
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
        ax.set_ylabel("Global Cost")
        ax.set_title("Global Cost Reduction")
        fig.savefig(f"{self.parent_path}/results/iteration_global_cost.png")

    def result_iteration_local_cost(self):
        results_data = self.__retrieve_file_data("local-cost")
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
        ax.set_ylabel("Average Local Cost")
        ax.set_title("Local Cost Reduction")
        fig.savefig(f"{self.parent_path}/results/iteration_local_cost.png")

    def result_changes_in_plans(self):
        result_data = self.__retrieve_file_data("selected-plans")
        headers = result_data[0]
        results = result_data[1:]
        #  Sort results into their runs i.e. the simulation they come from
        runs = {}
        for result in results:
            result = result.strip("\n").split(",")
            result = [int(float(i)) for i in result]
            if result[0] not in runs:
                runs[result[0]] = [result[1:]]
            else:
                runs[result[0]].append(result[1:])
        #  Ensure that the iterations are correctly sorted
        for run, iterations in runs.items():
            iterations.sort(key=lambda x: x[0])
        #  Compute the average changes between each iteration
        changes_on_each_iteration = [[] for _ in range(len(runs))]
        for run, iterations in runs.items():
            run_changes = []
            for i in range(1, len(iterations)-1):
                changes = 0.
                prev_iter = iterations[i-1][1:]
                cur_iter = iterations[i][1:]
                for p, c in zip(prev_iter, cur_iter):
                    if p != c:
                        changes += 1
                changes /= len(prev_iter)
                run_changes.append(changes)
            changes_on_each_iteration[run] = run_changes
        #  Compute the average change for each iteration
        final_changes = []
        for i in range(len(changes_on_each_iteration[0])):
            avg = 100.*(sum([arr[i] for arr in changes_on_each_iteration])/len(changes_on_each_iteration))
            final_changes.append(avg)
        #  Compute the maximum number of iterations
        iterations = np.array([i for i in range(1, len(final_changes)+1)])
        #  Plot!
        x = np.linspace(iterations.min(), iterations.max(), 1000)
        x_y_spline = make_interp_spline(iterations, final_changes)
        y = x_y_spline(x)

        fig, ax = plt.subplots()
        ax.plot(x, y)
        ax.grid()
        ax.set_xlabel("Iterations")
        ax.set_ylabel("% Change in Plans")
        ax.set_title("% Change in Plans Overtime")
        fig.savefig(f"{self.parent_path}/results/change_in_plans.png")

    def experiment_reorganisation_mismatch(self):
        pass

    def experiment_average_sensing(self):
        results = []
        for _ in range(0, 1):
            results.append(self.pg.generate_paths())
        #  Pre-defined sensing target, change to dynamic
        target = np.array([5., 9., 4., 5., 4., 8.])
        avg_sensing = np.zeros((1, 6))
        for result in results:
            total_sensing = np.zeros((1, 6))
            for plan in result:
                total_sensing += np.array(plan[1])
            avg_sensing += total_sensing
        avg_sensing /= len(results)
        avg_sensing = avg_sensing[0]
        error = abs(target - avg_sensing)
        #  Generate x-axis
        x = [i for i in range(1, len(avg_sensing)+1)]
        #  Plot!
        fig, ax = plt.subplots()
        ax.bar(x, target)
        ax.errorbar(x, target, yerr=error, fmt="o", color="r")
        ax.set_xlabel("Dimension")
        ax.set_ylabel("Average Sensing")
        ax.set_title("Average Sensing Error")
        fig.savefig(f"{self.parent_path}/results/average_sensing_error.png")


if __name__ == '__main__':
    pgr = PathGenerationResultsCollector()
    # pgr.collect_results()
    pgr.perform_experiments()
