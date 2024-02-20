import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E701'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(3)
    logconf.stop()
    print("Nun")


def simple_log(scf, logconf):
    with SyncLogger(scf, lg_stab) as logger:
        for log_entry in logger:
            print(log_entry)

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            return (data)

            # modified to loop this command
            for i in range(1,10):
            #while True:
                print(i)
                print('[%d][%s]: %s' % (timestamp, logconf_name, data))
                time.sleep(1)

            break

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    """
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        simple_log_async(scf, lg_stab)
    """
    while True:
        if (input("CONT") == "n"):
            break
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            #doesn't like running this line if you have ros running - resource busy

            #simple_connect()
            print(simple_log(scf,lg_stab)["stabilizer.roll"])

"""
class logAsFunction:cflib.crtp.init_drivers()
        self.lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self.lg_stab.add_variable('stabilizer.roll', 'float')
        self.lg_stab.add_variable('stabilizer.pitch', 'float')
        self.lg_stab.add_variable('stabilizer.yaw', 'float')self.uri[drone]
    def __init__(self, drones):
        self.drones = drones
    
    def log(self, elapsed):
        print("Logged")
"""
import os
import random


#Test function for logging

class logAsFunction:
    #Init function, initialise anything required for logging here
    def __init__(self, drones):
        print("Logging hardware")
        self.drones = drones
        #Creates necessary folders to hold test results in
        self.path = 'Results'
        try:
            os.mkdir(self.path)
        except:
            pass
        self.path = self.path+"/"+'droneLogger'
        try:
            os.mkdir(self.path)
        except:
            pass
        
        #Creates text files to write individual drone data to
        self.my_dict = {}
        self.uri = {}
        for drone in self.drones:
            dronePath = self.path+"/"+drone[len(drone) - 2:]
            self.my_dict[drone] = open(dronePath, 'w')
            self.my_dict[drone].write("time,battery%,state")
            #self.uri[drone] = 'radio://0/90/2M/E7E7E7E7'+drone
            self.uri[drone] = drone

    
    #Logging, takes time elapsed as variable
    def log(self, elapsed):
        #Can initialise anything else needed here
        print("Logging hard")
        for drone in self.drones:
            cflib.crtp.init_drivers()
            self.lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
            self.lg_stab.add_variable('pm.batteryLevel','uint8_t')
            self.lg_stab.add_variable('pm.state','int8_t')
            with SyncCrazyflie(self.uri[drone], cf=Crazyflie(rw_cache='./cache')) as scf:
                self.my_dict[drone].write("\n")
                print("SHED")
                with SyncLogger(scf, self.lg_stab) as logger:
                    for log_entry in logger:
 
                        data = log_entry[1]
                        self.my_dict[drone].write(str(elapsed)+","+str(data['pm.batteryLevel'])+","+str(data['pm.state']))
                        break
        print("Hardware logged")