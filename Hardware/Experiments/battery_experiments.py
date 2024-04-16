#from cdca_epos_executor import *
import os
import sys
import rospy
from std_msgs.msg import String
import time

CRAZYSWARM_SCRIPTS_FILE_PATH = "/home/adam/Documents/Packages/crazyswarm/ros_ws/src/crazyswarm/scripts"
HOVER_HEIGHT = 0.5
RATE=10 # Set to the same rate as the logging rostopic publishes
X_DISTANCE = 0.5533
Y_DISTANCE = 0.235

sys.path.append(CRAZYSWARM_SCRIPTS_FILE_PATH)
from pycrazyswarm import Crazyswarm

os.chdir(CRAZYSWARM_SCRIPTS_FILE_PATH)
swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
pub = None

IDs = [cf.id for cf in allcfs.crazyflies]
print("Number of drones: %s" % len(IDs))

def create_node():
    global pub

    try:
        rospy.init_node('chatter', anonymous=True)
    except:
        pass

    pub = rospy.Publisher('status_logger', String, queue_size=10)

    input("Logging setup complete\nPress any key to continue")

def log(id=None, msg=""):
    if id != None:
        print("Publishing %s with message %s" % (id, msg))
        pub.publish("Drone %s:%s" % (id, msg))
        rospy.loginfo("Drone %s:%s" % (id, msg))
    else:
        pub.publish("All drones:%s" % (msg))
        rospy.loginfo("All drones:%s" % (msg))

def log_all(ids, msg=""):
    for id in ids:
        log(id)

def setUp(dur, run=True):
    log(msg="Experiment start")
    if run == True:
        allcfs.takeoff(targetHeight=HOVER_HEIGHT, duration=dur)
    log(msg="Taking off")
    timeHelper.sleep(dur)
    log(msg="Hovering")

def tearDown(dur, run=True):
    log(msg="Preparing to land")
    if run == True:
        allcfs.land(targetHeight=0.05, duration=dur)
    log(msg="Landing")
    timeHelper.sleep(dur)
    log(msg="Experiment end")

def exp_takeOffLand(dur):
    setUp(dur)
    tearDown(dur)

def exp_Hover(dur, run=True):
    setUp(dur, run=run)

    # Hover until program killed
    inp = None
    while inp != "":
        inp = input("Press any key to end the experiment")
    #     log(1,msg="hovering")

    # while True:
    #     log(1,msg="hovering")
    #     time.sleep(1)

    tearDown(dur, run=run)

def move_x(setup_dur, speed, run=True):

    movement_duration = X_DISTANCE / speed # get speed from cdca code

    pos1 = [(0,2*Y_DISTANCE,HOVER_HEIGHT),(0,1*Y_DISTANCE,HOVER_HEIGHT),(0,0*Y_DISTANCE,HOVER_HEIGHT),(0,-1*Y_DISTANCE,HOVER_HEIGHT)]
    pos2 = [(X_DISTANCE,2*Y_DISTANCE,HOVER_HEIGHT),(X_DISTANCE,2*Y_DISTANCE,HOVER_HEIGHT),(X_DISTANCE,2*Y_DISTANCE,HOVER_HEIGHT),(X_DISTANCE,2*Y_DISTANCE,HOVER_HEIGHT)]

    rel_pos1 = (X_DISTANCE,0,0)
    rel_pos2 = (-X_DISTANCE,0,0)

    setUp(setup_dur,run=run)

    for i in range(0,len(allcfs.crazyflies)):
        if run == True:
            print("moving to initial positions")
            print(pos1[:len(IDs)][i])
            allcfs.crazyflies[i].goTo(pos1[:len(IDs)][i],0,movement_duration)
            timeHelper.sleep(movement_duration)

    # REPEAT
    try:
        while True:

            # All drones move one cell in the positive x direction
            for i in range(0,len(allcfs.crazyflies)):
                if run == True:
                    #allcfs.crazyflies[i].goTo(pos2[:len(IDs)][i],0,movement_duration) # see if you can do this with allcfs (i.e. they all move at the same time)
                    allcfs.crazyflies[i].goTo(rel_pos2,0,movement_duration, relative=True)
            log(msg="Moving to position 2")
            timeHelper.sleep(movement_duration)

            # LOG
            log(msg="Position 2")
            timeHelper.sleep(movement_duration)

            # All drones move back to their original positions
            for i in range(0,len(allcfs.crazyflies)):
                if run == True:
                    #allcfs.crazyflies[i].goTo(pos1[:len(IDs)][i],0,movement_duration) # see if you can do this with allcfs (i.e. they all move at the same time)
                    allcfs.crazyflies[i].goTo(rel_pos1,0,movement_duration, relative=True)
            log(msg="Moving to position 1")
            timeHelper.sleep(movement_duration)

            # LOG
            log(msg="Position 1")
            timeHelper.sleep(movement_duration)

    except KeyboardInterrupt:
        print("landing")
        tearDown(setup_dur, run=run)

#log_all(IDs)
#for i in range(0,10):
# while True:
#     log(1, msg="test")
#     time.sleep(0.5)

create_node()
#exp_Hover(3,run=False)
move_x(3,0.1,run=True)