import csv
#from Hardware.loggingAsFunction import *
from logTester import *
from loggingAsFunction import *
from logTestB import *
import time



class RecordingFactory:
    def __init__(self, tests, drones):
        self.tests = []
        for i in tests:
            #if i == "hardware":
            #    self.tests.append(logAsFunction)
            if i == "test":
                self.tests.append(logTest(drones))
            if i == "testB":
                self.tests.append(logTestB(drones))
            if i == "logAsFunction":
                self.tests.append(logAsFunction(drones))
                



def testLogging(drones, tests):
    loggers = RecordingFactory(tests, drones)
    counter  = 0
    interval = 2

    start = time.time()
    trueStart = time.time()
    while counter < 5:
        time.sleep(.5)
        current = time.time()
        if ((current - start) > interval):
            elapsed = round(current - trueStart, 2)
            print("Logging Results:")
            for i in loggers.tests:
                i.log(elapsed)
            start = time.time()
            counter+=1

def testThreadLogging(drones, tests):
    #loggers = RecordingFactory(tests, drones)
    logger = logAsFunction(drones)
    interval = 2
    status = True
    t1 = threading.Thread(target = logger.autoLog, args = (interval, lambda : status,))
    t1.start()
    while input("Continue") == "y":
        print("SHEDS")
    status = False
    t1.join()



#testLogging(["radio://0/90/2M/E7E7E7E704", "radio://0/80/2M/E7E7E7E701"], ["test", "testB","logAsFunction"])
#testLogging(["radio://0/80/2M/E7E7E7E701"], ["logAsFunction"])
#testLogging(["radio://0/90/2M/E7E7E7E704", "radio://0/80/2M/E7E7E7E701"], ["logAsFunction"])
testThreadLogging(["radio://0/90/2M/E7E7E7E704", "radio://0/80/2M/E7E7E7E701"], ["logAsFunction"])
