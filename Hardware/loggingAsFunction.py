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

#Test function for logging

class logAsFunction:
    #Init function, initialise anything required for logging here
    def __init__(self, drones):
        print("Logging hardware")
        self.drones = drones
        self.trueStart = time.time()
        self.WLock = threading.RLock() #Lock for file write process


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
    def log(self, elapsed= None):
        #Can initialise anything else needed here
        self.WLock.acquire()
        elapsed = round(time.time() - self.trueStart, 2)
        print("Logging hard")
        for drone in self.drones:
            cflib.crtp.init_drivers()
            self.lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
            self.lg_stab.add_variable('pm.batteryLevel','uint8_t')
            self.lg_stab.add_variable('pm.state','int8_t')
            with SyncCrazyflie(self.uri[drone], cf=Crazyflie(rw_cache='./cache')) as scf:
                self.my_dict[drone].write("\n")
                with SyncLogger(scf, self.lg_stab) as logger:
                    for log_entry in logger:
                        data = log_entry[1]
                        self.my_dict[drone].write(str(elapsed)+","+str(data['pm.batteryLevel'])+","+str(data['pm.state']))
                        break
        self.WLock.release()
    
    def autoLog(self, interval, killSwitch):
        prior = self.trueStart - interval
        while killSwitch():
            time.sleep(interval/4) #Do not remove, stop gap to stop thread nabbing resources
            current = time.time()
            if ((current - prior) > interval):
                self.WLock.acquire()
                elapsed = round(current - self.trueStart, 2)
                for drone in self.drones:
                    cflib.crtp.init_drivers()
                    self.lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
                    self.lg_stab.add_variable('pm.batteryLevel','uint8_t')
                    self.lg_stab.add_variable('pm.state','int8_t')
                    with SyncCrazyflie(self.uri[drone], cf=Crazyflie(rw_cache='./cache')) as scf:
                        self.my_dict[drone].write("\n")
                        with SyncLogger(scf, self.lg_stab) as logger:
                            for log_entry in logger:
        
                                data = log_entry[1]
                                self.my_dict[drone].write(str(elapsed)+","+str(data['pm.batteryLevel'])+","+str(data['pm.state']))
                                break

                self.WLock.release()
                prior = time.time()