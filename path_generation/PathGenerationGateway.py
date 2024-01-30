from JavaGatewayClient import JavaGatewayClient

from py4j.java_gateway import JavaGateway, GatewayParameters


class PathGenerationGateway(JavaGatewayClient):

    def execute(self) -> str:
        """ Execute the desired workflow from the JVM."""
        return ""

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        return 0
