from typing import List, Tuple
from path_generation.PathGenerationController import PathGenerationController
from pathlib import Path


class PathGenerator:

    def __init__(self):
        self._state = 0
        self._generation_manager = PathGenerationController()

    def get_state(self) -> int:
        return self._state

    def generate_paths(self) -> [List[Tuple[float, List[float]]], None]:
        # Generate num paths and return them
        result_code = self._generation_manager.generate_paths()
        if result_code != 0:
            return None
        result_code = self._generation_manager.select_plan()
        if result_code != 0:
            return None

        #  Get results from file
        result = self._generation_manager.extract_results()
        return result

    # Read the testbed mapping from a csv file
    # Extract the coordinate of the testbed from the file
    def read_testbed_mapping(self):
        parent_path = Path(__file__).parent.resolve()
        path_to_testbed = parent_path / 'PlanGeneration/datasets/testbed/testbed.csv'

        mapping = {'sense': [], 'base': []}
        with open(path_to_testbed, 'r') as f:
            next(f)  # Skip the first line
            for line in f:
                print(line)
                line = line.strip()
                if len(line) == 0:  # make sure the line is not empty
                    continue

                parts = line.split(',')
                object_type = parts[0].lower()
                if object_type in mapping:
                    mapping[object_type].append((int(parts[2]), int(parts[3])))

        # debug
        for object_type in mapping:
            for location in mapping[object_type]:
                print(f"{object_type}: {location}")
        # debug end
        return mapping

    def convert_data_to_table(self, plans):
        # Convert the plans to a 2D-array that consists of the location and the time
        # aID\t(s)  0    1    2  ...
        #       0   10,2 10,2 10,2
        #       1   0,50 1,48 1,21
        #       2   0,0   5,0 10,0  <- location of the agent at time t

        location_time = []  # 2D array

        return location_time
