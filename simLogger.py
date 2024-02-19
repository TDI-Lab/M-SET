import os
import time
import threading

#Test function for logging
class LogSim:
    #Crazyswaem object, whether to print or not, off is default
    def __init__(self, swarm, printer = False):
        print("Log Sim Enabled")
        self.swarm = swarm
        self.trueStart = time.time()
        self.printer = printer
        self.WLock = threading.RLock() #Lock for file write process

        #Creates necessary folders to hold test results in
        self.path = 'Results'
        try:
            os.mkdir(self.path)
        except:
            pass
        self.path = self.path
        try:
            os.mkdir(self.path)
        except:
            pass
        
        #Creates text files to write individual drone data to
        self.my_dict = {}
        dronePath = self.path+"/"+'simulated.txt'
        self.file = open(dronePath, 'w')
        self.file.write("time,positions")

        #Can initialise anything else needed here
    
    #Logging, takes time elapsed as variable
    def log(self):
        #print("Logged test Sim")
        msg = str(round(time.time() - self.trueStart, 2))+","+str(self.swarm.position())
        if self.printer:
            print(msg)
        self.WLock.acquire()
        self.file.write("\n")
        self.file.write(msg)
        self.WLock.release()

    def autoLog(self, interval, killSwitch):
        #print("Logged test Sim")

        prior = self.trueStart - interval
        while killSwitch():
            time.sleep(interval/4) #Do not remove, stop gap to stop thread nabbing resources
            current = time.time()
            if ((current - prior) > interval):
                elapsed = round(current - self.trueStart, 2)
                msg = str(elapsed)+","+str(self.swarm.position())
                if self.printer:
                    print(msg)
                self.WLock.acquire()
                self.file.write("\n")
                self.file.write(msg)
                self.WLock.release()
                prior = time.time()