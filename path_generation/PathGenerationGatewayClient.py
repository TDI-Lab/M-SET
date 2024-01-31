from JavaGatewayClient import JavaGatewayClient


class PathGenerationGatewayClient(JavaGatewayClient):

    def execute(self) -> int:
        """ Execute the desired workflow from the JVM."""
        return 0

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        return 0
