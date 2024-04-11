from copy import deepcopy
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import make_interp_spline

from path_generation.ConfigManager import ConfigManager
from path_generation.PathGenerator import PathGenerator


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
        plan_sizes = [2 ** i for i in range(1, 8)]
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