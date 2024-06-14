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

def setUp(dur=3, run=True):
    log(msg="Experiment start")
    if run == True:
        allcfs.takeoff(targetHeight=HOVER_HEIGHT, duration=dur)
    log(msg="Taking off")
    timeHelper.sleep(dur)
    log(msg="Hovering")

def tearDown(dur=3, run=True):
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
    print(run)
    setUp(dur, run=run)

    # Hover until program killed
    inp = None
    while inp != "":
        inp = input("Press any key to end the experiment")

    tearDown(dur, run=run)

def move_x(setup_dur, speed, run=True, factor=1):

    movement_duration = factor*X_DISTANCE / speed # get speed from cdca code

    pos1 = [(0,2*Y_DISTANCE,HOVER_HEIGHT),(0,1*Y_DISTANCE,HOVER_HEIGHT),(0,0*Y_DISTANCE,HOVER_HEIGHT),(0,-1*Y_DISTANCE,HOVER_HEIGHT)]
    pos2 = [(X_DISTANCE,2*Y_DISTANCE,HOVER_HEIGHT),(X_DISTANCE,2*Y_DISTANCE,HOVER_HEIGHT),(X_DISTANCE,2*Y_DISTANCE,HOVER_HEIGHT),(X_DISTANCE,2*Y_DISTANCE,HOVER_HEIGHT)]
    
    origin = (0,0,HOVER_HEIGHT)
    rel_pos1 = (factor*X_DISTANCE,0,0)
    rel_pos2 = (factor*-X_DISTANCE,0,0)

    setUp(setup_dur,run=run)

    if len(allcfs.crazyflies) == 0:
        if run == True:
            allcfs.crazyflies[i].goTo(origin,0,5)
            timeHelper.sleep(movement_duration)
    else:
        for i in range(0,len(allcfs.crazyflies)):
            if run == True:
                print("moving to initial positions")
                print(pos1[:len(IDs)][i])
                allcfs.crazyflies[i].goTo(pos1[:len(IDs)][i],0,5)
                timeHelper.sleep(movement_duration)

    # set up for factor > 1
    if factor > 1:
        for i in range(0,len(allcfs.crazyflies)):
            if run == True:
                allcfs.crazyflies[i].goTo((rel_pos1[0]/factor,rel_pos1[1],rel_pos1[2]),0,5, relative=True)
        log(msg="Moving to position 2")
        timeHelper.sleep(movement_duration)

        # LOG
        log(msg="Position 2")
        timeHelper.sleep(1)

    # REPEAT
    try:
        while True:
            # All drones move one cell in the positive x direction
            for i in range(0,len(allcfs.crazyflies)):
                if run == True:
                    allcfs.crazyflies[i].goTo(rel_pos2,0,movement_duration, relative=True)
            log(msg="Moving to position 2")
            timeHelper.sleep(movement_duration)

            # LOG
            log(msg="Position 2")
            timeHelper.sleep(1)

            # All drones move back to their original positions
            for i in range(0,len(allcfs.crazyflies)):
                if run == True:
                    allcfs.crazyflies[i].goTo(rel_pos1,0,movement_duration, relative=True)
            log(msg="Moving to position 1")
            timeHelper.sleep(movement_duration)

            # LOG
            log(msg="Position 1")
            timeHelper.sleep(1)

    except:
        print("landing")
        tearDown(setup_dur, run=run)

def land_at_origin(run=True):
    setUp(run=run)
    allcfs.crazyflies[0].goTo((0,0,HOVER_HEIGHT),0,5)
    timeHelper.sleep(5)
    tearDown(run=run)

def log_test():
    setUp(run=False)
    tearDown(run=False)

def main(run, experiments):
    create_node()

    print(run)

    if "log_test" in experiments:
        log_test()

    if "land_at_origin" in experiments:
        land_at_origin(run=run)

    if "hover" in experiments:
        exp_Hover(3,run=run)
    
    if "move_x" in experiments:
        move_x(3,0.1,run=run,factor=1)

if __name__ == '__main__':
    args = sys.argv
    offset=0

    if '--sim' in args:
        offset = 1

    run = (args[1+offset] == "True")
    experiments=[]
    for arg in args[2+offset:]:
        experiments.append(arg)

    main(run, experiments)