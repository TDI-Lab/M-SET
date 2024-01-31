import unittest
from os import listdir

from PathGenerationGatewayClient import PathGenerationGatewayClient
from EPOSGatewayClient import EPOSGatewayClient


class TestPGGateway(unittest.TestCase):

    def test_basic_access(self):
        pg_gateway = PathGenerationGatewayClient("127.0.0.1", 8080)
        pg_gateway.start()
        self.assertEquals(0, pg_gateway.get_gateway().assertAvailable())


class TestEPOSGateway(unittest.TestCase):

    def test_basic_access(self):
        epos_gateway = EPOSGatewayClient("127.0.0.1", 8081)
        epos_gateway.start()
        self.assertEquals(0, epos_gateway.get_gateway().assertAvailable())

    def test_for_single_output_directory(self):
        epos_gateway = EPOSGatewayClient("127.0.0.1", 8081)
        epos_gateway.start()
        epos_gateway.execute()
        self.assertEquals(1, len(listdir("../EPOS/output")))
