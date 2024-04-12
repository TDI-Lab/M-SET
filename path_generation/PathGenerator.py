from typing import List, Tuple, Dict
from path_generation.PathGenerationController import PathGenerationController
from pathlib import Path


class PathGenerator:

    def __init__(self):
        self.generation_manager = PathGenerationController()

    def generate_paths(self, raw=False) -> \
            [List[Tuple[float, List[float], List[str]]], Dict[int, List[Tuple[float, float, float]]], None]:
        """
        For parameter raw, if True, a list of sensing values at each location, along with the taken path is returned.
        If False, a list of coordinates is returned for each agent instead, where the ith element
        is the position the drone should be at time i.
        :param raw: Return the 'raw' values from EPOS.
        :return: n paths for n drones.
        """
        result_code = self.generation_manager.generate_paths()
        if result_code != 0:
            return None
        self.generation_manager.move_plans()
        result_code = self.generation_manager.select_plan()
        if result_code != 0:
            return None

        result = self.generation_manager.extract_results()
        if raw:
            return result
        return self.convert_data_to_table(result)

    # Read the testbed mapping from a csv file
    # Extract the coordinate of the testbed from the file
    def read_sensing_requirements(self, debug=False) -> Dict[str, List[Dict]]:
        """
        Read the testbed mapping from a csv file and extract the coordinate of the testbed from the file.
        :param debug: Debug mode
        :return: A list of base stations and sensing points for the sensing requirements.
        """
        path_to_testbed = self.generation_manager.config.get("global", "MissionFile")

        mapping = {'sense': [], 'base': []}
        with open(path_to_testbed, 'r') as f:
            next(f)  # Skip the first line
            for line in f:
                line = line.strip()
                if len(line) == 0:  # make sure the line is not empty
                    continue

                parts = line.split(',')
                object_type = parts[0].lower()
                if object_type in mapping:
                    loc_id = f"station{parts[1]}" if object_type == "base" else str(parts[1])
                    x = float(parts[2])
                    y = float(parts[3])
                    z = float(parts[4])
                    mapping[object_type].append({'id': loc_id, 'x': x, 'y': y, 'z': z})

        if debug:
            for object_type in mapping:
                for location in mapping[object_type]:
                    print(f"{object_type}: {location}")
        return mapping

    # Convert the plans to a 2D-array that consists of the location and the time
    # aID\t(s)  0    1    2  ...
    #       0   10,2 10,2 10,2
    #       1   0,50 1,48 1,21
    #       2   0,0   5,0 10,0  <- location of the agent at time t
    def convert_data_to_table(self, plans: List[Tuple[float, List[float], List[str]]]) -> (
            Dict)[int, List[Tuple[float, float, float]]]:
        """
        Convert the plans to a 2D-array that consists of the location and the time.
        :param plans: Drone plans extracted from EPOS
        :return:
        """
        sensing_map = self.read_sensing_requirements()
        paths = {}
        for agent_id, plan in enumerate(plans):
            movements = []
            for i, loc in enumerate(plan[2]):
                if i == 0 or i == len(plan[2])-1:
                    location = list(filter(lambda x: x["id"] == loc, sensing_map["base"]))[0]
                    sense_r = 1
                else:
                    location = list(filter(lambda x: x["id"] == loc, sensing_map["sense"]))[0]
                    sense_r = int(plan[1][int(location["id"])])
                x, y, z = location["x"], location["y"], location["z"]
                movements += [(x, y, z) for _ in range(0, sense_r)]
            paths[agent_id] = movements
        return paths
