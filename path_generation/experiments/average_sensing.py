from copy import deepcopy
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt

from path_generation.ConfigManager import ConfigManager


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