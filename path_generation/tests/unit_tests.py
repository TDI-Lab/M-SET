import unittest

from PathGenerationGateway import PathGenerationGateway
from EPOSGateway import EPOSGateway


class TestPGGateway(unittest.TestCase):

    def test_basic_access(self):
        pg_gateway = PathGenerationGateway("127.0.0.1", 8080)
        pg_gateway.start()
        self.assertEquals(0, pg_gateway.get_gateway().assertAvailable())


class TestEPOSGateway(unittest.TestCase):

    def test_basic_access(self):
        epos_gateway = EPOSGateway("127.0.0.1", 8081)
        epos_gateway.start()
        self.assertEquals(0, epos_gateway.get_gateway().assertAvailable())
