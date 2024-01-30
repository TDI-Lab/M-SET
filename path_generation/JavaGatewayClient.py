from py4j.java_gateway import GatewayParameters, JavaGateway


class JavaGatewayClient:

    def __init__(self, addr: str, port: int):
        self.__gateway = None
        self.__parameters = GatewayParameters(address=addr, port=port)

    def get_gateway(self) -> JavaGateway:
        return self.__gateway

    def start(self) -> int:
        """ Start the Gateway Server and load all necessary classes."""
        self.__gateway = JavaGateway(gateway_parameters=self.__parameters).entry_point
        return 0

    def execute(self) -> str:
        """ Execute the desired workflow from the JVM."""
        pass

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        pass
