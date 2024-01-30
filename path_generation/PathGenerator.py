from typing import List, Tuple
from PathGenerationController import PathGenerationController # import PathGenerationController_stub.py

class PathGenerator:
    # defining variable types
    state: int
    generationManager: 'PathGenerationController'

    def __init__(self) -> None:
        self.state = 0
        self.generationManager = PathGenerationController() # inherit from PathGenerationController()

    def getState(self) -> int:
        # Return the state of the PathGenerator
        return self.state

    def generatePaths(self, num: int) -> List[Tuple[float, List[float]]]:
        # Generate num paths and return them
        pass