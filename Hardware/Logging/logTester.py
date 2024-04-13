import os
import random


#Test function for logging

class logTest:
    #Init function, initialise anything required for logging here
    def __init__(self, drones):
        print("Log Test Enabled")
        self.drones = drones

        #Creates necessary folders to hold test results in
        self.path = 'Results'
        try:
            os.mkdir(self.path)
        except:
            pass
        self.path = self.path+"/"+'hardwareTests'
        try:
            os.mkdir(self.path)
        except:
            pass


        #Creates text files to write individual drone data to
        self.my_dict = {}
        for drone in self.drones:
            dronePath = self.path+"/"+drone
            self.my_dict[drone] = open(dronePath, 'w')
            self.my_dict[drone].write("time,battery%,position,state")

        #Can initialise anything else needed here
    
    #Logging, takes time elapsed as variable
    def log(self, elapsed):
        print("Logged test")
        for drone in self.drones:
            self.my_dict[drone].write("\n")
            self.my_dict[drone].write(str(elapsed)+","+str(random.randrange(1,100))+","+str(random.randrange(1,5))+","+str(random.randrange(0,5)))