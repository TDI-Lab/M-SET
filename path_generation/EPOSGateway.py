from JavaGatewayClient import JavaGatewayClient

from py4j.java_gateway import JavaGateway, GatewayParameters


class EPOSGateway(JavaGatewayClient):

    def execute(self) -> str:
        """ Execute the desired workflow from the JVM."""
        return self.__gateway.run()

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        return self.__gateway.stop()
