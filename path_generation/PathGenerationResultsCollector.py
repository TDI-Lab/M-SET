"""
Will eventually move these into a results factory when it's available.
"""
from copy import deepcopy
from distutils.dir_util import copy_tree
from os import listdir, mkdir
from os.path import isdir
from pathlib import Path
from pprint import pprint
from shutil import copyfile, copy2

import numpy as np
import matplotlib.pyplot as plt
from numpy import std
from scipy.interpolate import make_interp_spline

from path_generation.PathGenerator import PathGenerator
from path_generation.ConfigManager import ConfigManager
from path_generation.PlanGeneration.PlanGenerator import PlanGenerator


class PathGenerationResultsCollector:
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

    def __init__(self):
        self.parent_path = Path(__file__).parent.resolve()
        self.pg = PathGenerator()

    def collect_results(self):
        funcs = list(filter(lambda x: x[0:6] == "result", dir(PathGenerationResultsCollector)))
        for func in funcs:
            result_func = getattr(PathGenerationResultsCollector, func)
            result_func(self)

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

    def perform_experiments(self):
        funcs = list(filter(lambda x: x[0:10] == "experiment", dir(PathGenerationResultsCollector)))
        for func in funcs:
            result_func = getattr(PathGenerationResultsCollector, func)
            result_func(self)

    def __retrieve_file_data(self, filename, headers=False):
        output_dir = f"{self.parent_path}/EPOS/output"
        result_dir = listdir(output_dir)[0]
        results_file = f"{output_dir}/{result_dir}/{filename}.csv"
        with open(results_file) as file:
            lines = file.readlines()
        if headers:
            return lines
        return lines[1:]

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

    def result_iteration_global_cost(self):
        results_data = self.__retrieve_file_data("global-cost")
        results = results_data
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

    def result_changes_in_plans(self):
        result_data = self.__retrieve_file_data("selected-plans")
        results = result_data
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
            for i in range(1, len(iterations) - 1):
                changes = 0.
                prev_iter = iterations[i - 1][1:]
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
            avg = 100. * (sum([arr[i] for arr in changes_on_each_iteration]) / len(changes_on_each_iteration))
            final_changes.append(avg)
        #  Compute the maximum number of iterations
        iterations = np.array([i for i in range(1, len(final_changes) + 1)])
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

    def experiment_plan_generation_fault(self):
        #  Set target directories
        parent_path = Path(__file__).parent.resolve()
        testbed_dir = f"{parent_path}/PlanGeneration/datasets/testbed/"
        plan_gen_config_target = f"/{parent_path}/PlanGeneration/conf/generation.properties"
        epos_config_target = f"/{parent_path}/EPOS/conf/epos.properties"
        config = ConfigManager()
        #  Create testbed.csv
        # testbed = [
        #     "type,id,x,y,z,value",
        #     "SENSE,0,1,1,1,5",
        #     "SENSE,1,2,1,1,9",
        #     "SENSE,2,3,1,1,4",
        #     "SENSE,3,1,2,1,5",
        #     "SENSE,4,2,2,1,4",
        #     "SENSE,5,3,2,1,8",
        #     "BASE,0,0,0,0,0",
        #     "BASE,1,4,0,0,0",
        #     "BASE,2,4,3,0,0",
        #     "BASE,3,0,3,0,0"
        # ]
        # with open(f"{testbed_dir}/testbed.csv", "w") as file:
        #     for line in testbed:
        #         file.write(f"{line}\n")
        #  Ensure EPOS configuration is correct
        config.set_target_path(epos_config_target)
        epos_conf = deepcopy(self.EPOS_STANDARD_PROPERTIES)
        epos_conf["numPlans"] = 32
        epos_conf["globalSignalPath"] = "datasets/testbed/testbed.target"
        epos_conf["globalCostFunction"] = "MIS"
        config.write_config_file(epos_conf)
        #  Set up config for plan generation
        config.set_target_path(plan_gen_config_target)
        #  Enumerate Experiments
        experiment_configs = []
        base_battery = 2700
        for i in range(1, 100):
            ec = deepcopy(self.PLAN_GENERATION_STANDARD_PROPERTIES)
            ec["power"]["batteryCapacity"] = base_battery * i
            experiment_configs.append(ec)
        #  Run the experiment!
        battery_value = []
        avg_mismatch = []
        for experiment in experiment_configs:
            avg_error = 0.
            repeats = 5
            for i in range(0, repeats):
                #  1. Write config file
                config.write_config_file(experiment)
                #  2. Generate plans
                pg = PathGenerator()
                results = pg.generate_paths()
                # 3. Load target
                target_file = f"{testbed_dir}/testbed.target"
                goal = self.__load_target(target_file)
                #  4. Calculate Mismatch
                total = np.zeros(goal.shape)
                for result in results:
                    result = np.array(result[1])
                    total += result
                mismatch = sum(abs(goal - total))
                avg_error += mismatch
            avg_error /= repeats
            avg_mismatch.append(avg_error)
            battery_value.append(experiment["power"]["batteryCapacity"])
        #  Plot!
        battery_value = np.array(battery_value)
        avg_mismatch = np.array(avg_mismatch)

        x = np.linspace(battery_value.min(), battery_value.max(), 1000)
        x_y_spline = make_interp_spline(battery_value, avg_mismatch)
        y = x_y_spline(x)

        fig, ax = plt.subplots()
        ax.plot(x, y)
        ax.grid()
        ax.set_xlabel("Battery Capacity (J)")
        ax.set_ylabel("Average Sensing Mismatch")
        ax.set_title("Battery Related Sensing Mismatch")
        fig.savefig(f"{self.parent_path}/results/battery_capacity_sensing_mismatch.png")

    def experiment_reorganisation_mismatch(self):
        pass

    def experiment_average_sensing(self):
        #  Set target directories
        parent_path = Path(__file__).parent.resolve()
        testbed_dir = f"{parent_path}/PlanGeneration/datasets/testbed/"
        plan_gen_config_target = f"/{parent_path}/PlanGeneration/conf/generation.properties"
        epos_config_target = f"/{parent_path}/EPOS/conf/epos.properties"
        config = ConfigManager()
        #  Set up configurations
        config.set_target_path(plan_gen_config_target)
        config.write_config_file(self.PLAN_GENERATION_STANDARD_PROPERTIES)
        config.set_target_path(epos_config_target)
        epos_conf = deepcopy(self.EPOS_STANDARD_PROPERTIES)
        epos_conf["strategy"] = "convergence"
        epos_conf["globalSignalPath"] = "datasets/testbed/testbed.target"
        epos_conf["globalCostFunction"] = "MIS"
        #  Run experiment
        results = []
        for _ in range(0, 10):
            results.append(self.pg.generate_paths())
        #  Pre-defined sensing target, change to dynamic
        target = f"{testbed_dir}/testbed.target"
        target = self.__load_target(target)
        avg_sensing = np.zeros(target.shape)
        for result in results:
            total_sensing = np.zeros(target.shape)
            for plan in result:
                total_sensing += np.array(plan[1])
            avg_sensing += total_sensing
        avg_sensing /= len(results)
        error = abs(target - avg_sensing)
        #  Generate x-axis
        x = [i for i in range(1, len(avg_sensing) + 1)]
        #  Plot!
        fig, ax = plt.subplots()
        ax.bar(x, target)
        ax.errorbar(x, target, yerr=error, fmt="o", color="r")
        ax.set_xlabel("Dimension")
        ax.set_ylabel("Average Sensing")
        ax.set_title("Average Sensing Error")
        fig.savefig(f"{self.parent_path}/results/average_sensing_error.png")

    def experiment_plans_global_cost(self):
        #  Set target directories
        parent_path = Path(__file__).parent.resolve()
        testbed_dir = f"{parent_path}/PlanGeneration/datasets/testbed/"
        plan_gen_config_target = f"/{parent_path}/PlanGeneration/conf/generation.properties"
        epos_config_target = f"/{parent_path}/EPOS/conf/epos.properties"
        config = ConfigManager()
        #  Vary the number of cells visited by each agent per plan
        visited_num_start = 1
        visited_num_end = 6
        visited_cell_num = [i for i in range(visited_num_start, visited_num_end + 1)]
        fig, ax = plt.subplots(len(visited_cell_num), 1, figsize=(8, 7))
        for visited_num in visited_cell_num:
            #  Number of plans changing each experiment -> need to modify both plan gen and epos
            experiments = []
            base_plan_gen_conf = deepcopy(self.PLAN_GENERATION_STANDARD_PROPERTIES)
            base_plan_gen_conf["plan"]["maxVisitedCells"] = visited_num
            base_epos_conf = deepcopy(self.EPOS_STANDARD_PROPERTIES)
            base_epos_conf["numSimulations"] = 50
            base_epos_conf["strategy"] = "convergence"
            base_epos_conf["globalSignalPath"] = "datasets/testbed/testbed.target"
            base_epos_conf["globalCostFunction"] = "MIS"
            #  Generate experiments with different plan sizes
            plan_sizes = [2**i for i in range(1, 8)]
            for i in plan_sizes:
                new_pg_conf = deepcopy(base_plan_gen_conf)
                new_pg_conf["plan"]["planNum"] = i
                new_epos_conf = deepcopy(base_epos_conf)
                new_epos_conf["numPlans"] = i
                experiments.append((new_pg_conf, new_epos_conf))
            #  Perform experiment
            avg_final_costs = []
            avg_final_variance = []
            for pg_e, epos_e in experiments:
                config.set_target_path(plan_gen_config_target)
                config.write_config_file(pg_e)
                config.set_target_path(epos_config_target)
                config.write_config_file(epos_e)
                repeats = 1
                final_cost = []
                final_cost_variance = []
                for i in range(0, repeats):
                    pg = PathGenerator()
                    pg.generate_paths()
                    results_data = self.__retrieve_file_data("global-cost")
                    results = results_data
                    avg_costs = self.__compute_mean_several_runs(results)
                    avg_stds = self.__compute_std_several_runs(results)
                    final_cost.append(avg_costs[-1])
                    final_cost_variance.append(avg_stds[-1])
                final_cost = sum(final_cost) / len(final_cost)
                final_variance = sum(final_cost_variance) / len(final_cost_variance)
                avg_final_costs.append(final_cost)
                avg_final_variance.append(final_variance)
            #  Plot! (for this experiment)
            plan_sizes = np.array(plan_sizes)
            avg_final_costs = np.array(avg_final_costs)
            avg_final_variance = np.array(avg_final_variance)

            #  Interpolate mean
            x = np.linspace(plan_sizes.min(), plan_sizes.max(), len(plan_sizes) * 10)
            x_y_spline = make_interp_spline(plan_sizes, avg_final_costs)
            y = x_y_spline(x)
            #  Interpolate max variance
            x_y_spline = make_interp_spline(plan_sizes, avg_final_costs - avg_final_variance)
            y_min = x_y_spline(x)
            #  Interpolate min variance
            x_y_spline = make_interp_spline(plan_sizes, avg_final_costs + avg_final_variance)
            y_max = x_y_spline(x)

            ax_index = visited_num - visited_num_start
            ax[ax_index].plot(x, y)
            ax[ax_index].fill_between(x, y_min, y_max, color="b", alpha=.15)
            ax[ax_index].grid()
            ax[ax_index].set_ylabel(f"{visited_num}")
        fig.supxlabel("# Plans per Agent")
        fig.supylabel("Final Global Cost")
        fig.suptitle("Final Global Cost vs # Plans per Agent")
        fig.tight_layout()
        fig.savefig(f"{self.parent_path}/results/plans_global_cost.png")


if __name__ == '__main__':
    pgr = PathGenerationResultsCollector()
    pgr.experiment_average_sensing()
