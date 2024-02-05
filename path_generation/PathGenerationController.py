from os import listdir
from pprint import pprint
from typing import List, Tuple

from JavaGatewayClient import JavaGatewayClient
from PathGenerationGatewayClient import PathGenerationGatewayClient
from EPOSGatewayClient import EPOSGatewayClient

from decouple import config


class PathGenerationController:

    def __init__(self):
        self._pg_controller = PathGenerationGatewayClient(config("PG_IP"), config("PG_PORT", cast=int))
        self._epos_controller = EPOSGatewayClient(config("EPOS_IP"), config("EPOS_PORT", cast=int))

    def start_server(self) -> int:
        # Start the Java servers
        self._pg_controller.start()
        self._epos_controller.start()
        return 0

    def generate_paths(self, num: int) -> int:
        # Generate plans for each agent, where num is the number of agents
        result_code = self._pg_controller.execute()
        return result_code

    def select_plan(self) -> int:
        # Execute the EPOS Algorithm for plan selection
        result_code = self._epos_controller.execute()
        return result_code

    def extract_results(self) -> List[Tuple[float, List[float]]]:
        results_dir = listdir("EPOS/output")[0]
        selected_plans_file = f"EPOS/output/{results_dir}/selected-plans.csv"
        with open(selected_plans_file) as file:
            lines = file.readlines()
        plan_indexes = [int(index) for index in lines[-1].strip("\n").split(",")[2:]]
        selected_plans = []
        for i, index in enumerate(plan_indexes):
            with open(f"EPOS/datasets/Sense20000_64p_4b/agent_{i}.plans") as file:
                plans = file.readlines()
                selected_plan = plans[index-1].strip("\n")
                cost, plan = selected_plan.split(":")
                cost = float(cost)
                plan = [float(i) for i in plan.split(",")]
                selected_plans.append((cost, plan))
        return selected_plans
