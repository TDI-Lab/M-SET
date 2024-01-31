from typing import List, Tuple
from PathGenerationController import PathGenerationController  # import PathGenerationController_stub.py


class PathGenerator:

    def __init__(self):
        self._state = 0
        self._generation_manager = PathGenerationController()

    def get_state(self) -> int:
        return self._state

    def generate_paths(self, num: int) -> List[Tuple[float, List[float]]]:
        # Generate num paths and return them
        result_code = self._generation_manager.generate_paths(num)
        result_code = self._generation_manager.select_plan()
        #  Get results from file
        result = [(0., [])]
        return result
