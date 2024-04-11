"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import rospy 
import threading
import time
from simLogger import *

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


#use processes
def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    #print(cf.position(), type(cf.position()))
    
    status = True #Killswitch for autoLog thread
    logger = LogSim(cf, True) #swarm, True to print in terminal/ False otherwise
    #Threading function args: interval to log at(seconds), killswitch call
    t1 = threading.Thread(target = logger.autoLog, args = (1, lambda : status,))
    t1.start()

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    logger.log() #Log at chosen time
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)

    status = False #Kill function
    print("FINISHED")
    t1.join()


if __name__ == "__main__":
    main()
