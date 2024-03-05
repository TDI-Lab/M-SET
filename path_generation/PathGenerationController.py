import configparser
from os import listdir
from pathlib import Path
from pprint import pprint
from typing import List, Tuple
from distutils.dir_util import copy_tree

from path_generation.ConfigManager import ConfigManager
from path_generation.PlanGeneration.PlanGenerator import PlanGenerator
from path_generation.EPOS.EPOSWrapper import EPOSWrapper

import decouple


class PathGenerationController:

    def __init__(self):
        self.parent_path = Path(__file__).parent.resolve()
        self.config = configparser.ConfigParser()
        self.config.read(f"{self.parent_path}/../drone_sense.properties")
        self.config_generator = ConfigManager()

        self._pg_controller = None
        self._epos_controller = None

    def __construct_plan_generation_properties(self):
        properties = {
            "plan": {
                "dataset": "testbed",
                "planNum": self.config.get("path_generation", "NumberOfPlans"),
                "agentsNum": self.config.get("global", "NumberOfDrones"),
                "timeSlots": 0,
                "maxVisitedCells": self.config.get("path_generation", "MaximumNumberOfVisitedCells")
            },
            "map": {
                "stationsNum": 4,
                "height": 1,
                "mapLength": 4
            },
            "power": {
                "batteryCapacity": self.config.get("drone", "BatteryCapacity"),
                "bodyMass": self.config.get("drone", "BodyMass"),
                "batteryMass": self.config.get("drone", "BatteryMass"),
                "rotorNum": self.config.get("drone", "NumberOfRotors"),
                "rotorDia": self.config.get("drone", "RotorDiameter"),
                "projectedBody": self.config.get("drone", "ProjectedBodyArea"),
                "projectedBattery": self.config.get("drone", "ProjectedBatteryArea"),
                "powerEfficiency": self.config.get("drone", "PowerEfficiency"),
                "groundSpeed": self.config.get("drone", "GroundSpeed"),
                "airDensity": self.config.get("environment", "AirDensity"),
                "airSpeed": self.config.get("drone", "AirSpeed")
            }
        }
        return properties

    def generate_paths(self) -> int:
        # Create config files from drone_sense.properties
        self.config_generator.set_target_path(f"{self.parent_path}/PlanGeneration/conf/generation.properties")
        plan_gen_properties = self.__construct_plan_generation_properties()
        self.config_generator.write_config_file(plan_gen_properties)
        # Generate plans for each agent, where num is the number of agents
        self._pg_controller = PlanGenerator()
        self._pg_controller.clean_datasets()
        result_code = self._pg_controller.generate_plans(False)
        return result_code

    # Move the generated plans to the EPOS directory
    def move_plans(self):
        plan_gen_dir = f'{self.parent_path}/PlanGeneration/datasets/{self.config.get("plan", "dataset")}'
        epos_dir = f'{self.parent_path}/EPOS/datasets/{self.config.get("plan", "dataset")}'
        copy_tree(plan_gen_dir, epos_dir)

    # Execute the EPOS Algorithm for plan selection
    def select_plan(self) -> int:
        self.config_generator.write_epos_config_file()
        self._epos_controller = EPOSWrapper()
        result_code = self._epos_controller.run()
        self.move_plans()
        return result_code

    def __get_plan_indexes(self):
        # read selected-plan.csv from the first directory within the output directory
        results_dir = listdir(f"{self.parent_path}/EPOS/output")[0]
        selected_plans_file = f"{self.parent_path}/EPOS/output/{results_dir}/selected-plans.csv"
        with open(selected_plans_file) as file:
            lines = file.readlines()
        # extract plan indexes from the last line of the file, converting them from strings to integers
        plan_indexes = [int(index) for index in lines[-1].strip("\n").split(",")[2:]]
        return plan_indexes

    def __extract_sensing_values_from_indexes(self, indexes):
        #  Extract sensing values at each step of the path
        selected_plans = []
        for i, index in enumerate(indexes):
            with open(f"{self.parent_path}/EPOS/datasets/{self.config.get('plan', 'dataset')}/agent_{i}.plans") as file:
                plans = file.readlines()
                selected_plan = plans[index - 1].strip("\n")
                cost, plan = selected_plan.split(":")
                cost = float(cost)
                plan = [float(i) for i in plan.split(",")]
                selected_plans.append((cost, plan))
        return selected_plans

    def __extract_paths_from_indexes(self, indexes):
        #  Extract sensing values at each step of the path
        selected_paths = []
        for i, index in enumerate(indexes):
            with open(f"{self.parent_path}/EPOS/datasets/{self.config.get('plan', 'dataset')}/agent_{i}.paths") as file:
                paths = file.readlines()
                selected_path = paths[index - 1].strip("\n").split(",")
                selected_paths.append(selected_path)
        return selected_paths

    def extract_results(self) -> List[Tuple[float, List[float], List[str]]]:
        # Extract the results from the selected plans and paths
        plan_indexes = self.__get_plan_indexes()
        print(f"Selected plan indexes: {plan_indexes}")
        selected_plans = self.__extract_sensing_values_from_indexes(plan_indexes)
        selected_paths = self.__extract_paths_from_indexes(plan_indexes)
        results = []
        for plan, path in zip(selected_plans, selected_paths):
            results.append((plan[0], plan[1], path))
        return results
