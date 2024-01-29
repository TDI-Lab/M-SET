from JavaServer import JavaServer

from py4j.java_gateway import JavaGateway


class PathGenerationServer(JavaServer):

    def __init__(self):
        self.pg_server = None

    def start(self) -> int:
        """ Start the Gateway Server and load all necessary classes."""
        pass

    def execute(self) -> str:
        """ Execute the desired workflow from the JVM."""
        pass

    def stop(self) -> int:
        """ Stop the Gateway Server."""
        pass
