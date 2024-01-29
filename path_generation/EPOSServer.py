from JavaServer import JavaServer

from py4j.java_gateway import JavaGateway, GatewayParameters


class EPOSServer(JavaServer):

    def __init__(self):
        self.epos_server = None

    def start(self) -> int:
        """ Start the Gateway Server and load all necessary classes."""
        parameters = GatewayParameters(address="127.0.0.1", port=8081)
        self.epos_server = JavaGateway(gateway_parameters=parameters).entry_point
        return 0

    def execute(self) -> str:
        """ Execute the desired workflow from the JVM."""
        return self.epos_server.start()

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        return self.epos_server.stop()


if __name__ == "__main__":
    epos_server = EPOSServer()
    epos_server.start()
    print(epos_server.execute())
    print(epos_server.stop())
