import configparser
import shutil
from distutils.dir_util import copy_tree
from distutils.util import strtobool
from os import listdir, mkdir
from pathlib import Path
from shutil import copy2
from typing import List, Tuple
from itertools import groupby

from path_generation.ConfigManager import ConfigManager
from path_generation.EPOS.EPOSWrapper import EPOSWrapper
from path_generation.PlanGeneration.PlanGenerator import PlanGenerator

#  TODO: Add support for multiple EPOS simulations
#  TODO: Add support for creating plans for single drone systems


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
                "dataset": self.config.get("global", "MissionName"),
                "planNum": self.config.get("path_generation", "NumberOfPlans"),
                "agentsNum": self.config.get("global", "NumberOfDrones"),
                "timeSlots": 0,
                "pathMode": self.config.get("path_generation", "PathMode"),
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

    def __construct_epos_properties(self):
        mission_name = self.config.get('global', 'MissionName')
        with open(f"{self.parent_path}/EPOS/datasets/{mission_name}/{mission_name}.target", "r") as file:
            target_vector_length = len(file.readlines()[0].split(","))
        properties = {
            "dataset": mission_name,
            "numSimulations": self.config.get("epos", "NumberOfSimulations"),
            "numIterations": self.config.get("epos", "IterationsPerSimulation"),
            "numAgents": self.config.get("global", "NumberOfDrones"),
            "numPlans": self.config.get("path_generation", "NumberOfPlans"),
            "numChildren": self.config.get("epos", "NumberOfChildren"),
            "planDim": target_vector_length,
            "shuffle": self.config.get("epos", "Shuffle"),
            "shuffle_file": self.config.get("epos", "ShuffleFile"),
            "numberOfWeights": self.config.get("epos", "NumberOfWeights"),
            "weightsString": self.config.get("epos", "WeightsString"),
            "behaviours": self.config.get("epos", "behaviours"),
            "agentsBehaviourPath": self.config.get("epos", "agentsBehaviourPath"),
            "constraint": self.config.get("epos", "constraint"),
            "constraintPlansPath": self.config.get("epos", "constraintPlansPath"),
            "constraintCostsPath": self.config.get("epos", "constraintCostsPath"),
            "strategy": self.config.get("epos", "strategy"),
            "periodically.reorganizationPeriod": self.config.get("epos", "periodically.reorganizationPeriod"),
            "convergence.memorizationOffset": self.config.get("epos", "convergence.memorizationOffset"),
            "globalCost.reductionThreshold": self.config.get("epos", "globalCost.reductionThreshold"),
            "strategy.reorganizationSeed": self.config.get("epos", "strategy.reorganizationSeed"),
            "globalSignalPath":
                f"{self.parent_path}/EPOS/datasets/{mission_name}/{mission_name}.target",
            "globalCostFunction": self.config.get("epos", "globalCostFunction"),
            "scaling": self.config.get("epos", "scaling"),
            "localCostFunction": self.config.get("epos", "localCostFunction"),
            "logger.GlobalCostLogger": "true",
            "logger.LocalCostMultiObjectiveLogger": "true",
            "logger.TerminationLogger": "true",
            "logger.SelectedPlanLogger": "true",
            "logger.GlobalResponseVectorLogger": "true",
            "logger.PlanFrequencyLogger": "true",
            "logger.UnfairnessLogger": "true",
            "logger.GlobalComplexCostLogger": "true",
            "logger.WeightsLogger": "true",
            "logger.ReorganizationLogger": "true",
            "logger.VisualizerLogger": "false",
            "logger.PositionLogger": "true",
            "logger.HardConstraintLogger": "true"
        }
        return properties

    def generate_paths(self) -> int:
        # Create config files from drone_sense.properties
        self.config_generator.set_target_path(f"{self.parent_path}/PlanGeneration/conf/generation.properties")
        plan_gen_properties = self.__construct_plan_generation_properties()
        self.config_generator.write_config_file(plan_gen_properties)
        # Generate plans for each agent
        self._pg_controller = PlanGenerator()
        self._pg_controller.clean_datasets()
        #  Move mission to datasets folder
        mission_file = self.config.get("global", "MissionFile")
        result_code = self._pg_controller.generate_plans(mission_file=mission_file)
        return result_code

    # Move the generated plans to the EPOS directory
    def move_plans(self):
        dataset_name = self.config.get("global", "MissionName")
        plan_gen_dir = f'{self.parent_path}/PlanGeneration/datasets/{dataset_name}'
        epos_dir = f'{self.parent_path}/EPOS/datasets/{dataset_name}'
        copy_tree(plan_gen_dir, epos_dir)

    def __select_for_single_drone_system(self, results_dir):
        with open(f"{results_dir}/termination.csv", "x") as file:
            file.write("Run,Terminal Iteration\n")
            file.write("0,1")
        with open(f"{results_dir}/selected-plans.csv", "x") as file:
            file.write("Run,Iteration,agent-0\n")
            file.write("0,0,0")

    # Execute the EPOS Algorithm for plan selection
    def select_plan(self) -> int:
        self._epos_controller = EPOSWrapper()
        self.config_generator.set_target_path(f"{self.parent_path}/EPOS/conf/epos.properties")
        epos_properties = self.__construct_epos_properties()
        if epos_properties["numAgents"] == "1":
            output_dir = f"{self.parent_path}/EPOS/output"
            self._epos_controller.clean_output(output_dir)
            results_dir = f"{output_dir}/{self.config.get('global', 'MissionName')}_result"
            mkdir(results_dir)
            self.__select_for_single_drone_system(results_dir)
            return 0
        else:
            self.config_generator.write_config_file(epos_properties)
            show_out = bool(strtobool(self.config.get("epos", "EPOSstdout")))
            show_err = bool(strtobool(self.config.get("epos", "EPOSstderr")))
            result_code = self._epos_controller.run(out=show_out, err=show_err)
            return result_code

    def __get_plan_indexes(self):
        # read selected-plan.csv from the first directory within the output directory
        results_dir = listdir(f"{self.parent_path}/EPOS/output")[0]
        termination_iters = f"{self.parent_path}/EPOS/output/{results_dir}/termination.csv"
        with open(termination_iters) as file:
            lines = file.readlines()
        termination_indexes = [int(i.strip("\n").split(",")[1]) for i in lines[1:]]
        selected_plans_file = f"{self.parent_path}/EPOS/output/{results_dir}/selected-plans.csv"
        with open(selected_plans_file) as file:
            lines = file.readlines()
        #  extract plans from each simulation, and select the "best" index
        lines = [list(map(int, line.strip("\n").split(","))) for line in lines[1:]]
        seperated_simulations = [list(group) for k, group in groupby(lines, lambda x: x[0])]
        indexes = {}
        for termination_index, simulation in zip(termination_indexes, seperated_simulations):
            termination_indexes = tuple(simulation[termination_index-1][2:])
            if termination_indexes not in indexes:
                indexes[termination_indexes] = 1
            else:
                indexes[termination_indexes] += 1
        indexes = list(indexes.items())
        plan_indexes = list(max(indexes, key=lambda x: x[1])[0])
        return plan_indexes

    def __extract_sensing_values_from_indexes(self, indexes):
        #  Extract sensing values at each step of the path
        selected_plans = []
        data_dir = self.config.get('global', 'MissionName')
        for i, index in enumerate(indexes):
            with open(f"{self.parent_path}/EPOS/datasets/{data_dir}/agent_{i}.plans") as file:
                plans = file.readlines()
                selected_plan = plans[index].strip("\n")
                cost, plan = selected_plan.split(":")
                cost = float(cost)
                plan = [float(i) for i in plan.split(",")]
                selected_plans.append((cost, plan))
        return selected_plans

    def __extract_paths_from_indexes(self, indexes):
        #  Extract sensing values at each step of the path
        selected_paths = []
        data_dir = self.config.get('global', 'MissionName')
        for i, index in enumerate(indexes):
            with open(f"{self.parent_path}/EPOS/datasets/{data_dir}/agent_{i}.paths") as file:
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
