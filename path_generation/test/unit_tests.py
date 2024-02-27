import unittest

from path_generation.PathGenerator import PathGenerator


class TestPathConverterSingleAgent(unittest.TestCase):

    def test_0s_plan(self):
        #  Arrange
        pg = PathGenerator()
        plans = [(0.0, [0., 0., 0., 0., 0., 0.], ["station0", "station0"])]
        #  Action
        result = pg.convert_data_to_table(plans)
        #  Assert
        desired_result = [(0., 0., 0.), (0., 0., 0.)]
        self.assertEquals(result, desired_result)

    def test_choppy_plan(self):
        #  Arrange
        pg = PathGenerator()
        plans = [(0.0, [10., 0., 5., 0., 0., 1.], ["station0", "0", "2", "5", "station0"])]
        #  Action
        result = pg.convert_data_to_table(plans)
        #  Assert
        desired_result = [
            (0., 0., 0.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.),
            (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (3., 1., 1.), (3., 1., 1.), (3., 1., 1.),
            (3., 1., 1.), (3., 1., 1.), (3., 2., 1.), (0., 0., 0.)
        ]
        self.assertEquals(result, desired_result)

    def test_moving_plan(self):
        #  Arrange
        pg = PathGenerator()
        plans = [(0.0, [1., 1., 1., 1., 1., 1.], ["station2", "5", "4", "0", "1", "2", "3", "station2"])]
        #  Action
        result = pg.convert_data_to_table(plans)
        #  Assert
        desired_result = [
            (4., 3., 0.), (3., 2., 1.), (2., 2., 1.), (1., 1., 1.),
            (2., 1., 1.), (3., 1., 1.), (1., 2., 1.), (4., 3., 0.)
        ]
        self.assertEquals(result, desired_result)
