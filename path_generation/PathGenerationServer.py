from JavaServer import JavaServer

from py4j.java_gateway import JavaGateway, GatewayParameters


class PathGenerationServer(JavaServer):

    def __init__(self):
        self.pg_server = None

    def start(self) -> int:
        """ Start the Gateway Server and load all necessary classes."""
        parameters = GatewayParameters(address="127.0.0.1", port=8080)
        self.pg_server = JavaGateway(gateway_parameters=parameters).entry_point
        return 0

    def execute(self) -> str:
        """ Execute the desired workflow from the JVM."""
        return self.pg_server.start()

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        return self.pg_server.stop()


if __name__ == "__main__":
    pg = PathGenerationServer()
    pg.start()
    print(pg.execute())
    print(pg.stop())
