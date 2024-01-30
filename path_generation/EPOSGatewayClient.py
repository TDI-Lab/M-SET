from JavaGatewayClient import JavaGatewayClient


class EPOSGatewayClient(JavaGatewayClient):

    def execute(self) -> str:
        """ Execute the desired workflow from the JVM."""
        return self._gateway.run()

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        return self._gateway.stop()
