import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

cflib.crtp.init_drivers()

lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
lg_stab.add_variable('stabilizer.roll', 'float')
lg_stab.add_variable('stabilizer.pitch', 'float')
lg_stab.add_variable('stabilizer.yaw', 'float')

def log_stab_callback(timestamp, data, logconf):
    return (timestamp, logconf.name, data)

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)

    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()


with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    simple_log_async(scf, lg_stab)

class logAsFunction:
    def __init__(self, drones):
        self.drones = drones
    
    def log(self, elapsed):
        print("Logged")
