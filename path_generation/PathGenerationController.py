import configparser
from os import listdir
from pathlib import Path
from pprint import pprint
from typing import List, Tuple
from distutils.dir_util import copy_tree

from PlanGeneration.PlanGenerator import PlanGenerator
from EPOSGatewayClient import EPOSGatewayClient

import decouple


class PathGenerationController:

    def __init__(self):
        self.parent_path = Path(__file__).parent.resolve()
        self.config = configparser.ConfigParser()
        self.config.read(f'{self.parent_path}/PlanGeneration/conf/generation.properties')

        self._pg_controller = PlanGenerator()
        self._epos_controller = EPOSGatewayClient(decouple.config("EPOS_IP"), decouple.config("EPOS_PORT", cast=int))

    def start_server(self) -> int:
        # Start the Java servers
        self._epos_controller.start()
        return 0

    def generate_paths(self) -> int:
        # Generate plans for each agent, where num is the number of agents
        result_code = self._pg_controller.generate_plans(False)
        return result_code

    def move_plans(self):
        plan_gen_dir = f'{self.parent_path}/PlanGeneration/datasets/{self.config.get("plan", "dataset")}'
        epos_dir = f'{self.parent_path}/EPOS/datasets/{self.config.get("plan", "dataset")}'
        copy_tree(plan_gen_dir, epos_dir)

    def select_plan(self) -> int:
        # Execute the EPOS Algorithm for plan selection
        result_code = self._epos_controller.execute()
        self.move_plans()
        return result_code

    def extract_results(self) -> List[Tuple[float, List[float]]]:
        results_dir = listdir(f"{self.parent_path}/EPOS/output")[0]
        selected_plans_file = f"{self.parent_path}/EPOS/output/{results_dir}/selected-plans.csv"
        with open(selected_plans_file) as file:
            lines = file.readlines()
        plan_indexes = [int(index) for index in lines[-1].strip("\n").split(",")[2:]]
        selected_plans = []
        for i, index in enumerate(plan_indexes):
            with open(f"{self.parent_path}/EPOS/datasets/{self.config.get('plan', 'dataset')}/agent_{i}.plans") as file:
                plans = file.readlines()
                selected_plan = plans[index-1].strip("\n")
                cost, plan = selected_plan.split(":")
                cost = float(cost)
                plan = [float(i) for i in plan.split(",")]
                selected_plans.append((cost, plan))
        return selected_plans
