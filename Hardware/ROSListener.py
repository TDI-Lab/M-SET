#!/usr/bin/env python
import rospy
from std_msgs.msg import String
#from crazyflie_driver.msg import GenericLogData
#from crazyswarm import GenericLogData
from crazyswarm.msg import GenericLogData
from geometry_msgs.msg import Pose
#from pycrazyswarm import Crazyswarm
#print(Crazyswarm.msg)

def callback(data):
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.values)
   rospy.loginfo(data.values)

#https://forum.bitcraze.io/viewtopic.php?t=5190
#https://github.com/USC-ACTLab/crazyswarm/discussions/566
#rostopics echo /cf1/log1
   #http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
#https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/logparam/
   '''
   rostopic bw     display bandwidth used by topic
rostopic echo   print messages to screen
rostopic hz     display publishing rate of topic    
rostopic list   print information about active topics
rostopic pub    publish data to topic
rostopic type   print topic type'''
def listener():
   # In ROS, nodes are uniquely named. If two nodes with the same
   # name are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.
   rospy.init_node('listener', anonymous=True)

   rospy.Subscriber("/cf4/log1", GenericLogData, callback) # # MAKE SURE RIGHT CHANNEL IS SET

        # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

if __name__ == '__main__':
    listener()
    
    
   #rosbag record -a /cf1/log1
   #rostopic echo /cf1/log1 -b bagFileName.bag -p > fileName.csv
   
   
 #/home/adam/Documents/Packages/crazyswarm/ros_ws/src/crazyswarm/msg/GenericLogData.msg