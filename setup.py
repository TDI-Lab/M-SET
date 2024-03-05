"""
Run once to set up environment.
"""
from collections import OrderedDict

parameters = OrderedDict({
    "global": {
        "MissionName": "testbed",
        "MissionFile": "testbed.csv",
        "NumberOfDrones": 4
    },

    "path_generation": {
        "NumberOfPlans": 8,
        "MaximumNumberOfVisitedCells": 6
    },

    "epos": {
        "EPOSstdout": False,
        "EPOSstderr": False,
        "NumberOfChildren": 2,
        "PlanDimension": 6,
        "Shuffle": 0,
        "ShuffleFile": "permuation.csv",
        "NumberOfWeights": 2,
        "WeightsString": "0.0,0.0",
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
    },

    "drone": {
        "BatteryCapacity": 2700,
        "BodyMass": 0.027,
        "BatteryMass": 0.005,
        "NumberOfRotors": 4,
        "RotorDiameter": 0.03,
        "ProjectedBodyArea": 0.0599,
        "ProjectedBatteryArea": 0.0037,
        "PowerEfficiency": 1.25,
        "GroundSpeed": 6.94,
        "AirSpeed": 8.5
    },

    "environment": {
        "AirDensity": 1.225
    }
})

with open("drone_sense.properties", "w") as file:
    for section, section_params in parameters.items():
        file.write(f"[{section}]\n")
        for key, item in section_params.items():
            file.write(f"{key}={item}\n")
        file.write("\n")
