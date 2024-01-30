from JavaServer import JavaServer

from py4j.java_gateway import JavaGateway, GatewayParameters


class PathGenerationServer(JavaServer):

    def __init__(self):
        self.pg_server = None

    def start(self) -> int:
        """ Start the Gateway Server and load all necessary classes."""
        parameters = GatewayParameters(address="127.0.0.1", port=8080)
        self.pg_server = JavaGateway(gateway_parameters=parameters).entry_point
        return 0

    def execute(self) -> str:
        """ Execute the desired workflow from the JVM."""
        return ""

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        return 0
