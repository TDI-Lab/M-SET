from JavaGatewayClient import JavaGatewayClient


class EPOSGatewayClient(JavaGatewayClient):

    def execute(self) -> int:
        """ Execute the desired workflow from the JVM."""
        return self._gateway.entry_point.run()

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        return self._gateway.shutdown()
