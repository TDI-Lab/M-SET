"""
Run once to set up environment.
"""
from collections import OrderedDict

global_parameters = {
    "MissionFile": "testbed.csv",
    "NumberOfDrones": 4
}

path_generation_parameters = OrderedDict({
    #  Plan Generation
    "NumberOfPlans": 8,
    "MaximumNumberOfVisitedCells": 6,

    #  Drone Specification
    "BatteryCapacity": 2700,
    "BodyMass": 0.027,
    "BatteryMass": 0.005,
    "NumberOfRotors": 4,
    "RotorDiameter": 0.03,
    "ProjectedBodyArea": 0.0599,
    "ProjectedBatteryArea": 0.0037,
    "PowerEfficiency": 1.25,
    "GroundSpeed": 6.94,
    "AirSpeed": 8.5,

    #  Environment
    "AirDensity": 1.225
})

with open("drone_sense.properties", "w") as file:
    for key, item in path_generation_parameters.items():
        file.write(f"{key}={item}\n\n")
