from JavaGatewayClient import JavaGatewayClient   # import JavaServer_stub.py

class PathGenerationController:
    # defining variable types
    pgController: 'JavaGatewayClient'
    eposController: 'JavaGatewayClient'

    def __init__(self) -> None:
        self.pgController = JavaGatewayClient()
        self.eposController = JavaGatewayClient()

    def startServer(self) -> int:
        # Start the Java servers
        return 0

    def generatePaths(self, num: int) -> int:
        # Generate num paths and store them
        return 0

    def selectPlan(self) -> int:
        # Select a plan from the database
        return 0

    def stopServer(self) -> int:
        # Stop the Java servers
        return 0