import csv
#from Hardware.loggingAsFunction import *
from Hardware.logTester import *
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
                



def testLogging(drones, tests):
    loggers = RecordingFactory(tests, drones)
    counter  = 0
    interval = 5

    start = time.time()
    trueStart = time.time()
    while counter < 5:
        time.sleep(.5)
        current = time.time()
        if ((current - start) > interval):
            elapsed = round(current - trueStart, 0)
            print("Logging Results:")
            for i in loggers.tests:
                i.log(elapsed)
            start = time.time()
            counter+=1

testLogging(["01", "02", "03"], ["test", "testB"])