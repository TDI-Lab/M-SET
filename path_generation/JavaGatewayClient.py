from py4j.java_gateway import JavaGateway, GatewayParameters


class JavaGatewayClient:

    def __init__(self, addr: str, port: int):
        self._gateway = None
        self._parameters = GatewayParameters(address=addr, port=port)

    def get_gateway(self) -> JavaGateway:
        return self._gateway

    def start(self) -> int:
        """ Start the Gateway Server and load all necessary classes."""
        self._gateway = JavaGateway(gateway_parameters=self._parameters)
        return 0

    def execute(self) -> str:
        """ Execute the desired workflow from the JVM."""
        pass

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        self._gateway.shutdown()
        return 0
