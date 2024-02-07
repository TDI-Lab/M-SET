from py4j.java_gateway import JavaGateway, GatewayParameters


class EPOSGatewayClient:

    def __init__(self, addr: str, port: int):
        self._gateway = None
        self._parameters = GatewayParameters(address=addr, port=port)

    def get_gateway(self) -> JavaGateway:
        return self._gateway

    def start(self) -> int:
        """ Start the Gateway Server and load all necessary classes."""
        self._gateway = JavaGateway(gateway_parameters=self._parameters)
        return 0

    def execute(self) -> int:
        """ Execute the desired workflow from the JVM."""
        self._gateway.entry_point.run()
        return 0

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        return self._gateway.shutdown()
