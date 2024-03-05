"""
Run once to set up environment.
"""
from collections import OrderedDict

parameters = OrderedDict({
    "global": {
        "MissionFile": "testbed.csv",
        "NumberOfDrones": 4
    },

    "path_generation": {
        "NumberOfPlans": 8,
        "MaximumNumberOfVisitedCells": 6
    },

    "epos": {
        "EPOSstdout": False,
        "EPOSerrout": False
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
