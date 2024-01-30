import unittest

from PathGenerationServer import PathGenerationServer
from EPOSServer import EPOSServer


class TestPGServer(unittest.TestCase):

    def test_basic_access(self):
        pg_gateway = PathGenerationServer()
        pg_gateway.start()
        self.assertEquals(0, pg_gateway.pg_server.assertAvailable())


class TestEPOSServer(unittest.TestCase):

    def test_basic_access(self):
        epos_gateway = EPOSServer()
        epos_gateway.start()
        self.assertEquals(0, epos_gateway.epos_server.assertAvailable())
