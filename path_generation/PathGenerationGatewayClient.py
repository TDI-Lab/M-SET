from JavaGatewayClient import JavaGatewayClient


class PathGenerationGatewayClient(JavaGatewayClient):

    def execute(self) -> str:
        """ Execute the desired workflow from the JVM."""
        return ""

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        return 0
