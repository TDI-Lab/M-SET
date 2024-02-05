from typing import List, Tuple
from PathGenerationController import PathGenerationController  # import PathGenerationController_stub.py


class PathGenerator:

    def __init__(self):
        self._state = 0
        self._generation_manager = PathGenerationController()

    def get_state(self) -> int:
        return self._state

    def generate_paths(self, num: int) -> [List[Tuple[float, List[float]]], None]:
        # start the gateways
        self._generation_manager.start_server()
        # Generate num paths and return them
        result_code = self._generation_manager.generate_paths(num)
        if result_code != 0:
            return None
        result_code = self._generation_manager.select_plan()
        if result_code != 0:
            return None
        #  Get results from file
        result = self._generation_manager.extract_results()
        return result


if __name__ == '__main__':
    pg = PathGenerator()
    results = pg.generate_paths(4)
    for plan in results:
        print(plan)

