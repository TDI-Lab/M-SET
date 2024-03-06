import configparser
from distutils.dir_util import copy_tree
from distutils.util import strtobool
from os import listdir
from pathlib import Path
from typing import List, Tuple

from path_generation.ConfigManager import ConfigManager
from path_generation.EPOS.EPOSWrapper import EPOSWrapper
from path_generation.PlanGeneration.PlanGenerator import PlanGenerator


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

    def __construct_epos_properties(self):
        mission_name = self.config.get('global', 'MissionName')
        properties = {
            "dataset": mission_name,
            "numSimulations": self.config.get("epos", "NumberOfSimulations"),
            "numIterations": self.config.get("epos", "IterationsPerSimulation"),
            "numAgents": self.config.get("global", "NumberOfDrones"),
            "numPlans": self.config.get("path_generation", "NumberOfPlans"),
            "numChildren": self.config.get("epos", "NumberOfChildren"),
            "planDim": self.config.get("epos", "PlanDimension"),
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
            "logger.GlobalCostLogger": self.config.get("epos", "logger.GlobalCostLogger"),
            "logger.LocalCostMultiObjectiveLogger": self.config.get("epos", "logger.LocalCostMultiObjectiveLogger"),
            "logger.TerminationLogger": self.config.get("epos", "logger.TerminationLogger"),
            "logger.SelectedPlanLogger": self.config.get("epos", "logger.SelectedPlanLogger"),
            "logger.GlobalResponseVectorLogger": self.config.get("epos", "logger.GlobalResponseVectorLogger"),
            "logger.PlanFrequencyLogger": self.config.get("epos", "logger.PlanFrequencyLogger"),
            "logger.UnfairnessLogger": self.config.get("epos", "logger.UnfairnessLogger"),
            "logger.GlobalComplexCostLogger": self.config.get("epos", "logger.GlobalComplexCostLogger"),
            "logger.WeightsLogger": self.config.get("epos", "logger.WeightsLogger"),
            "logger.ReorganizationLogger": self.config.get("epos", "logger.ReorganizationLogger"),
            "logger.VisualizerLogger": self.config.get("epos", "logger.VisualizerLogger"),
            "logger.PositionLogger": self.config.get("epos", "logger.PositionLogger"),
            "logger.HardConstraintLogger": self.config.get("epos", "logger.HardConstraintLogger")
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
        dataset_name = self.config.get("global", "MissionName")
        plan_gen_dir = f'{self.parent_path}/PlanGeneration/datasets/{dataset_name}'
        epos_dir = f'{self.parent_path}/EPOS/datasets/{dataset_name}'
        copy_tree(plan_gen_dir, epos_dir)

    # Execute the EPOS Algorithm for plan selection
    def select_plan(self) -> int:
        self._epos_controller = EPOSWrapper()
        self.config_generator.set_target_path(f"{self.parent_path}/EPOS/conf/epos.properties")
        epos_properties = self.__construct_epos_properties()
        self.config_generator.write_config_file(epos_properties)
        show_out = bool(strtobool(self.config.get("epos", "EPOSstdout")))
        show_err = bool(strtobool(self.config.get("epos", "EPOSstderr")))
        result_code = self._epos_controller.run(out=show_out, err=show_err)
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
        data_dir = self.config.get('global', 'MissionName')
        for i, index in enumerate(indexes):
            with open(f"{self.parent_path}/EPOS/datasets/{data_dir}/agent_{i}.plans") as file:
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