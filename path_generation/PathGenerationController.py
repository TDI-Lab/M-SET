from JavaGatewayClient import JavaGatewayClient
from PathGenerationGatewayClient import PathGenerationGatewayClient
from EPOSGatewayClient import EPOSGatewayClient

from decouple import config


class PathGenerationController:

    def __init__(self):
        self._pg_controller = PathGenerationGatewayClient(config("PG_IP"), config("PG_PORT"))
        self._epos_controller = EPOSGatewayClient(config("EPOS_IP"), config("EPOS_PORT"))

    def start_server(self) -> int:
        # Start the Java servers
        pass

    def generate_paths(self, num: int) -> int:
        # Generate plans for each agent, where num is the number of agents
        result_code = self._pg_controller.execute()
        return result_code

    def select_plan(self) -> int:
        # Execute the EPOS Algorithm for plan selection
        pass

    def stop_server(self) -> int:
        # Stop the Java servers
        pass
