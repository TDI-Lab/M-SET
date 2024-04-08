from cdca_epos_executor import *

from pycrazyswarm import Crazyswarm
import numpy as np
#rom pycrazyswarm import CrazyflieServer

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
MASK=1
X_DISTANCE = 0.5533
Y_DISTANCE = 0.47 # check that's 0.235*2
X_DURATION=3
Y_DURATION=3
IN_SIMULATION = True

CRAZYSWARM_SCRIPTS_FILE_PATH = "/home/adam/Documents/Packages/crazyswarm/ros_ws/src/crazyswarm/scripts"
os.chdir(CRAZYSWARM_SCRIPTS_FILE_PATH)

swarm = Crazyswarm()
#server = CrazyflieServer()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs

for drone in allcfs.crazyflies:
    drone.setGroupMask(MASK)

drone_uris = return_uris((80,90),(2,3))

def exp_takeoffland(drone_uris, sim):
    n=0
    while not timeHelper.isShutdown():
        try:
            print("N: ",n)
            if sim == False:
                log_all_drones(drone_uris, ("battery"))
            allcfs.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION, groupMask=MASK)
            timeHelper.sleep(TAKEOFF_DURATION)# + HOVER_DURATION)
            allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION, groupMask=MASK)
            timeHelper.sleep(TAKEOFF_DURATION)
            n+=1
        except:
            log_all_drones(drone_uris, ("battery"))

def exp_hover(drone_uris, sim):
    n=0
    while not timeHelper.isShutdown():
        try:
            print("N: ",n)
            if sim == False:
                log_all_drones(drone_uris, ("battery"))
            allcfs.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION, groupMask=MASK)
            timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
            allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION, groupMask=MASK)
            timeHelper.sleep(TAKEOFF_DURATION)
            n+=1
        except:
            log_all_drones(drone_uris, ("battery"))

def exp_move_y(drone_uris, sim):
    positions = [
    [(-1*X_DISTANCE,0*Y_DISTANCE,HOVER_HEIGHT),(-0.5*X_DISTANCE,0*Y_DISTANCE,HOVER_HEIGHT),(0*X_DISTANCE,0*Y_DISTANCE,HOVER_HEIGHT),(0.5*X_DISTANCE,0*Y_DISTANCE,HOVER_HEIGHT)],
    [(-1*X_DISTANCE,1*Y_DISTANCE,HOVER_HEIGHT),(-0.5*X_DISTANCE,1*Y_DISTANCE,HOVER_HEIGHT),(0*X_DISTANCE,1*Y_DISTANCE,HOVER_HEIGHT),(0.5*X_DISTANCE,1*Y_DISTANCE,HOVER_HEIGHT)]
    ]

    n = 0
    while not timeHelper.isShutdown():
        try:
            print("N: ",n)
            if sim == False:
                log_all_drones(drone_uris, ("battery"))

            allcfs.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION, groupMask=MASK)
            timeHelper.sleep(TAKEOFF_DURATION)
            for i in range(0,len(allcfs.crazyflies)):
                #print(positions[n%2][i])
                allcfs.crazyflies[i].goTo(positions[n%2][i],0,Y_DURATION)
            timeHelper.sleep(Y_DURATION)
            allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION, groupMask=MASK)
            timeHelper.sleep(TAKEOFF_DURATION)

            n += 1

        except:
            log_all_drones(drone_uris, ("battery"))

def exp_move_x(drone_uris, sim):
    positions = [
    [(0*X_DISTANCE,-1*Y_DISTANCE,HOVER_HEIGHT),(0*X_DISTANCE,-0.5*Y_DISTANCE,HOVER_HEIGHT),(0*X_DISTANCE,0*Y_DISTANCE,HOVER_HEIGHT),(0*X_DISTANCE,0.5*Y_DISTANCE,HOVER_HEIGHT)],
    [(1*X_DISTANCE,-1*Y_DISTANCE,HOVER_HEIGHT),(1*X_DISTANCE,-0.5*Y_DISTANCE,HOVER_HEIGHT),(1*X_DISTANCE,0*Y_DISTANCE,HOVER_HEIGHT),(1*X_DISTANCE,0.5*Y_DISTANCE,HOVER_HEIGHT)]
    ]

    n = 0
    while not timeHelper.isShutdown():
        try:
            print("N: ",n)
            if sim == False:
                log_all_drones(drone_uris, ("battery"))

            allcfs.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION, groupMask=MASK)
            timeHelper.sleep(TAKEOFF_DURATION)
            for i in range(0,len(allcfs.crazyflies)):
                #print(positions[n%2][i])
                allcfs.crazyflies[i].goTo(positions[n%2][i],0,X_DURATION)
            timeHelper.sleep(X_DURATION)
            allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION, groupMask=MASK)
            timeHelper.sleep(TAKEOFF_DURATION)

            n += 1

        except:
            log_all_drones(drone_uris, ("battery"))

exp_takeoffland(drone_uris, IN_SIMULATION)
# exp_hover(drone_uris, IN_SIMULATION)
# exp_move_y(drone_uris,IN_SIMULATION)
# exp_move_x(drone_uris,IN_SIMULATION)