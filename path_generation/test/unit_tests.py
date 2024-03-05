import unittest

from path_generation.PathGenerator import PathGenerator


class TestPathConverter(unittest.TestCase):

    def test_0s_plan(self):
        #  Arrange
        pg = PathGenerator()
        plans = [(0.0, [0., 0., 0., 0., 0., 0.], ["station0", "station0"])]
        #  Action
        result = pg.convert_data_to_table(plans)
        #  Assert
        desired_result = {0: [(0., 0., 0.), (0., 0., 0.)]}
        self.assertEquals(result, desired_result)

    def test_choppy_plan(self):
        #  Arrange
        pg = PathGenerator()
        plans = [(0.0, [10., 0., 5., 0., 0., 1.], ["station0", "0", "2", "5", "station0"])]
        #  Action
        result = pg.convert_data_to_table(plans)
        #  Assert
        desired_result = {
            0: [(0., 0., 0.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.),
                (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (3., 1., 1.), (3., 1., 1.), (3., 1., 1.),
                (3., 1., 1.), (3., 1., 1.), (3., 2., 1.), (0., 0., 0.)]
        }
        self.assertEquals(result, desired_result)

    def test_moving_plan(self):
        #  Arrange
        pg = PathGenerator()
        plans = [(0.0, [1., 1., 1., 1., 1., 1.], ["station2", "5", "4", "0", "1", "2", "3", "station2"])]
        #  Action
        result = pg.convert_data_to_table(plans)
        #  Assert
        desired_result = {
            0: [(4., 3., 0.), (3., 2., 1.), (2., 2., 1.), (1., 1., 1.),
                (2., 1., 1.), (3., 1., 1.), (1., 2., 1.), (4., 3., 0.)]
        }
        self.assertEquals(result, desired_result)

    def test_multiple_agents_0s_plan(self):
        #  Arrange
        pg = PathGenerator()
        plans = [
            (0.0, [0., 0., 0., 0., 0., 0.], ["station0", "station0"]),
            (0.0, [0., 0., 0., 0., 0., 0.], ["station1", "station1"]),
            (0.0, [0., 0., 0., 0., 0., 0.], ["station2", "station2"]),
            (0.0, [0., 0., 0., 0., 0., 0.], ["station3", "station3"])
        ]
        #  Action
        result = pg.convert_data_to_table(plans)
        #  Assert
        desired_result = {
            0: [(0., 0., 0.), (0., 0., 0.)],
            1: [(4., 0., 0.), (4., 0., 0.)],
            2: [(4., 3., 0.), (4., 3., 0.)],
            3: [(0., 3., 0.), (0., 3., 0.)]
        }
        self.assertEquals(result, desired_result)

    def test_multiple_agents_choppy_plan(self):
        #  Arrange
        pg = PathGenerator()
        plans = [
            (0.0, [10., 0., 5., 0., 0., 1.], ["station0", "0", "2", "5", "station0"]),
            (0.0, [0., 3., 0., 5., 5., 0.], ["station1", "4", "3", "1", "station1"]),
            (0.0, [0., 2., 10., 0., 4., 0.], ["station2", "2", "1", "4", "station2"]),
            (0.0, [0., 0., 5., 0., 2., 1.], ["station3", "5", "4", "2", "station3"])
        ]
        #  Action
        result = pg.convert_data_to_table(plans)
        #  Assert
        desired_result = {
            0: [(0., 0., 0.),
                (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (1., 1., 1.),
                (1., 1., 1.), (1., 1., 1.), (1., 1., 1.),
                (3., 1., 1.), (3., 1., 1.), (3., 1., 1.), (3., 1., 1.), (3., 1., 1.),
                (3., 2., 1.),
                (0., 0., 0.)],
            1: [(4., 0., 0.),
                (2., 2., 1.), (2., 2., 1.), (2., 2., 1.), (2., 2., 1.), (2., 2., 1.),
                (1., 2., 1.), (1., 2., 1.), (1., 2., 1.), (1., 2., 1.), (1., 2., 1.),
                (2., 1., 1.), (2., 1., 1.), (2., 1., 1.),
                (4., 0., 0.)],
            2: [(4., 3., 0.),
                (3., 1., 1.), (3., 1., 1.), (3., 1., 1.), (3., 1., 1.), (3., 1., 1.), (3., 1., 1.), (3., 1., 1.),
                (3., 1., 1.), (3., 1., 1.), (3., 1., 1.),
                (2., 1., 1,), (2., 1., 1,),
                (2., 2., 1.), (2., 2., 1.), (2., 2., 1.), (2., 2., 1.),
                (4., 3., 0.)],
            3: [(0., 3., 0.),
                (3., 2., 1.),
                (2., 2., 1.), (2., 2., 1.),
                (3., 1., 1.), (3., 1., 1.), (3., 1., 1.), (3., 1., 1.), (3., 1., 1.),
                (0., 3., 0.)]
        }
        self.assertEquals(result, desired_result)

    def test_multiple_agents_moving_plan(self):
        #  Arrange
        pg = PathGenerator()
        plans = [
            (0.0, [1., 1., 1., 1., 1., 1.], ["station2", "5", "4", "0", "1", "2", "3", "station2"]),
            (0.0, [1., 1., 1., 1., 1., 1.], ["station1", "3", "2", "1", "4", "0", "5", "station1"]),
            (0.0, [1., 1., 1., 1., 1., 1.], ["station0", "2", "1", "4", "5", "3", "0", "station0"]),
            (0.0, [1., 1., 1., 1., 1., 1.], ["station3", "1", "5", "3", "0", "4", "2", "station3"])
        ]
        #  Action
        result = pg.convert_data_to_table(plans)
        #  Assert
        desired_result = {
            0: [(4., 3., 0.), (3., 2., 1.), (2., 2., 1.), (1., 1., 1.),
                (2., 1., 1.), (3., 1., 1.), (1., 2., 1.), (4., 3., 0.)],
            1: [(4., 0., 0.), (1., 2., 1.), (3., 1., 1.), (2., 1., 1.),
                (2., 2., 1.), (1., 1., 1.), (3., 2., 1.), (4., 0., 0.)],
            2: [(0, 0., 0.), (3., 1., 1.), (2., 1., 1.), (2., 2., 1.),
                (3., 2., 1.), (1., 2., 1.), (1., 1., 1.), (0., 0., 0.)],
            3: [(0., 3., 0.), (2., 1., 1.), (3., 2., 1.), (1., 2., 1.),
                (1., 1., 1.), (2., 2., 1.), (3., 1., 1.), (0., 3., 0.)]
        }
        self.assertEquals(result, desired_result)

    def test_multiple_agents_different_length_plan(self):
        #  Arrange
        pg = PathGenerator()
        plans = [
            (0.0, [1., 1., 0., 0., 1., 1.], ["station2", "5", "4", "0", "1", "station2"]),
            (0.0, [1., 1., 1., 1., 1., 0.], ["station1", "3", "2", "1", "4", "0", "station1"]),
            (0.0, [0., 1., 1., 0., 1., 0.], ["station0", "2", "1", "4", "station0"]),
            (0.0, [1., 1., 1., 1., 1., 1.], ["station3", "1", "5", "3", "0", "4", "2", "station3"])
        ]
        #  Action
        result = pg.convert_data_to_table(plans)
        #  Assert
        desired_result = {
            0: [(4., 3., 0.), (3., 2., 1.), (2., 2., 1.), (1., 1., 1.),
                (2., 1., 1.), (4., 3., 0.)],
            1: [(4., 0., 0.), (1., 2., 1.), (3., 1., 1.), (2., 1., 1.),
                (2., 2., 1.), (1., 1., 1.), (4., 0., 0.)],
            2: [(0, 0., 0.), (3., 1., 1.), (2., 1., 1.), (2., 2., 1.), (0., 0., 0.)],
            3: [(0., 3., 0.), (2., 1., 1.), (3., 2., 1.), (1., 2., 1.),
                (1., 1., 1.), (2., 2., 1.), (3., 1., 1.), (0., 3., 0.)]
        }
        self.assertEquals(result, desired_result)
