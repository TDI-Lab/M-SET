#from cdca_epos_executor import *
import os
import sys
import rospy
from std_msgs.msg import String
import time

CRAZYSWARM_SCRIPTS_FILE_PATH = "/home/adam/Documents/Packages/crazyswarm/ros_ws/src/crazyswarm/scripts"
HOVER_HEIGHT = 0.5
RATE=10 # Set to the same rate as the logging rostopic publishes
X_DISTANCE = 0
Y_DISTANCE = 0

sys.path.append(CRAZYSWARM_SCRIPTS_FILE_PATH)
from pycrazyswarm import Crazyswarm

os.chdir(CRAZYSWARM_SCRIPTS_FILE_PATH)
swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
pub = None

IDs = [cf.id for cf in allcfs.crazyflies]
print(len(IDs))

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
        pub.publish("Drone %s: %s." % (id, msg))
        rospy.loginfo("Drone %s: %s." % (id, msg))
    else:
        pub.publish("All drones: %s." % (msg))
        rospy.loginfo("All drones: %s." % (msg))

def log_all(ids, msg=""):
    for id in ids:
        log(id)

def setUp(dur):
    log(msg="Experiment start")
    allcfs.takeoff(targetHeight=HOVER_HEIGHT, duration=dur)
    timeHelper.sleep(dur)
    log(msg="All drones taken off")

def tearDown(dur):
    log(msg="All drones preparing to land")
    allcfs.land(targetHeight=0.05, duration=dur)
    timeHelper.sleep(dur)
    log(msg="Experiment end")

def exp_takeOffLand(dur):
    setUp(dur)
    tearDown(dur)

def exp_Hover(dur):
    setUp(dur)

    # Hover until program killed
    inp = None
    while inp != "":
        inp = input("Press any key to end the experiment")
    #     log(1,msg="hovering")

    # while True:
    #     log(1,msg="hovering")
    #     time.sleep(1)

    tearDown(dur)

def move_x(setup_dur, speed):
    setUp(setup_dur)

    movement_duration = X_DISTANCE / speed # get speed from cdca code

    pos1 = [(),(),(),()]
    pos2 = [(),(),(),()]

    # All drones move one cell in the positive x direction
    # goTo(pos2[:len(IDs)],0,movement_duration) # see if you can do this with allcfs (i.e. they all move at the same time)
    timeHelper.sleep(movement_duration)

    # All drones move one cell in the positive x direction
    # goTo(pos1[:len(IDs)],0,movement_duration) # see if you can do this with allcfs (i.e. they all move at the same time)
    timeHelper.sleep(movement_duration)

    tearDown(setup_dur)

#log_all(IDs)
#for i in range(0,10):
# while True:
#     log(1, msg="test")
#     time.sleep(0.5)

create_node()
exp_Hover(3)