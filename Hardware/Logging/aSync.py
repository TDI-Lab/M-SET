import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


import os
import random
import threading

""""
# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E702'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))



def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()


#def asyncLogger():

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='stateEstimate', period_in_ms=10)
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        simple_log_async(scf, lg_stab)

"""

class aSync:
    #Init function, initialise anything required for logging here
    def __init__(self, drones):
        logging.basicConfig(level=logging.ERROR)
        cflib.crtp.init_drivers()
        self.data = None
        self.drones = drones

        self.lg_stab = LogConfig(name='stateEstimate', period_in_ms=10)
        # self.lg_stab.add_variable('stateEstimate.x', 'float')
        # self.lg_stab.add_variable('stateEstimate.y', 'float')
        # self.lg_stab.add_variable('stateEstimate.z', 'float')
        self.lg_stab.add_variable('pm.batteryLevel', 'float')

    def runCallback(self):
        print("connecting to: ",self.drones[0])
        with SyncCrazyflie(self.drones[0], cf=Crazyflie(rw_cache='./cache')) as scf:

            self.simple_log_async(scf, self.lg_stab)

    #def log_stab_callback(timestamp, data, logconf):
    #    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

    def log_stab_callback(self, timestamp, data, logconf):
        self.data = data
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))

    def simple_log_async(self, scf, logconf):
        cf = scf.cf
        cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(self.log_stab_callback)
        logconf.start()
        time.sleep(0.02)
        logconf.stop()