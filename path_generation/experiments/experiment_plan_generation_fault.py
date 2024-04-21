from copy import deepcopy
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import make_interp_spline

from path_generation.ConfigManager import ConfigManager
from path_generation.PathGenerator import PathGenerator


def experiment_plan_generation_fault(self):
    #  Set target directories
    parent_path = Path(__file__).parent.resolve()
    testbed_dir = f"{parent_path}/PlanGeneration/datasets/testbed/"
    plan_gen_config_target = f"/{parent_path}/PlanGeneration/conf/generation.properties"
    epos_config_target = f"/{parent_path}/EPOS/conf/epos.properties"
    config = ConfigManager()
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
