"""
Adam Pearce
Built following this tutorial: https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_connect_log_param/
"""

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Imports for logging
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E704')
# MAKE SURE THE CHANNEL IS CORRECT HERE e.g. 80 or 90 usually
# MAKE SURE THE NUMBER IS CORRECT HERE e.g. E7E7E7E701 for drone 1, 02 at the end instead for drone 2 etc.


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def simple_connect():

    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")

def simple_log(scf, logconf):
    with SyncLogger(scf, lg_stab) as logger:
        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            # modified to loop this command
            for i in range(1,10):
                print('[%d][%s]: %s' % (timestamp, logconf_name, data))
                time.sleep(1)

            break

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Logging config
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        
    # Add variables you want to log
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
        
        # Variable for logging battery level
        # vbat referenced as being for battery here https://github.com/USC-ACTLab/crazyswarm/issues/148
        # vbat data type found here (ctrl-f for vbat) https://github.com/USC-ACTLab/crazyswarm/discussions/433
    lg_stab.add_variable('pm.vbat','float')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        #doesn't like running this line if you have ros running - resource busy

        #simple_connect()
        simple_log(scf,lg_stab)
