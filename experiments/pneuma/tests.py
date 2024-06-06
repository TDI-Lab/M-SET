import unittest
import pandas as pd
import os
from pnuema_experiments import PnuemaExperiments

class TestPnuemaExperiments(unittest.TestCase):
    def setUp(self):

        self.data_path = 'C:/Users/Alex/Downloads/20181024_d1_0830_0900 (1).csv'
        # Create a PnuemaExperiments object with the test data
        self.pnuema_experiments = PnuemaExperiments(self.data_path)

  

    def test_read_data(self):
        # Call the read_data method
        self.pnuema_experiments.read_data()

        # Check that the full_data attribute is a DataFrame with the expected columns
        self.assertIsInstance(self.pnuema_experiments.full_data, pd.DataFrame)
        self.assertListEqual(list(self.pnuema_experiments.full_data.columns), ['track_id', 'type', 'traveled_d', 'avg_speed', 'lat_1', 'lon_1', 'speed_1', 'lon_acc_1', 'lat_acc_1', 'time_1'])

if __name__ == '__main__':
    unittest.main()