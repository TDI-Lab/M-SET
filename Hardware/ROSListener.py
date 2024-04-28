#!/usr/bin/env python
import rospy
import sys
import time
import os
from std_msgs.msg import String
from crazyswarm.msg import GenericLogData
from geometry_msgs.msg import Pose
import numpy as np
try:
   from Hardware_constants import *
except:
   from Hardware.Hardware_constants import *

sys.path.append(CRAZYSWARM_SCRIPTS_FILE_PATH)
from pycrazyswarm import Crazyswarm

output_file = LOG_OUTPUT_FILE
print(os.getcwd())
try:
   file = open("Hardware Results/%s" % output_file, "w")
except:
   print("WARNING: LOG OUTPUT FILE NOT FOUND")
start = time.time()

#current = os.getcwd()
#os.chdir(CRAZYSWARM_SCRIPTS_FILE_PATH)

#ndrones=len(Crazyswarm().allcfs.crazyflies)
#status = np.full((1,ndrones),"idle")
status = ["idle", "idle", "idle", "idle"]

#os.chdir(current)

count=0
c=0
def callback(data,args):
   global c
   global count
   global status
   topic = args[0]
   ndrones = args[1]
   id = args[2]

   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.values)
   #print("topic=%s" % topic)

   #print(topic)

   if topic=="status":
      rospy.loginfo(data.data)
      #file.write(str(data.data))
      if id != None:
         msg = data.data.split(':')[min(1,len(data.data.split(':'))-1)] 
         status[int(id)-1] = msg
      else:
         for s in status:
            s = data.data
   else:
      rospy.loginfo("%s: Drone %s: Data: %s" % (count, id, data.values))
      file.write("\n")
      #file.write(str(count)+"/?/"+str(id)+"/?/"+str(data.values))
      #file.write(str(time.time()-start)+"/?/"+str(count)+"/?/"+str(id)+"/?/"+str(data.values))
      file.write(str(",".join((str(time.time()-start),str(count),str(id),str(data.values),"\""+str(status[int(id)-1])+"\""))))
      #file.write(str(data.values))
      c+=1

   count = c // ndrones
   #print(data.values) # This also works to just print it out to the cmd, but idk if it does different things in the background

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

def create_node():
   try:
      rospy.init_node('listener', anonymous=True)
   except:
      pass

def listener(ids=[1]):
   # In ROS, nodes are uniquely named. If two nodes with the same
   # name are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.
   
   global count

   # init node is called by the crazyswarm api when that is initialised, therefore errors if you call it again
   # But if running ROSListener.py directly (without crazyswarm) then you still need to run this line 
   create_node()

   for id in ids:
      rospy.Subscriber("/cf%s/log1" %id, GenericLogData, callback, ["log1", len(ids), id]) # # MAKE SURE RIGHT CHANNEL IS SET
   
      rospy.Subscriber("status_logger", String, callback, ["status", len(ids), id])
   #rospy.Subscriber("chatter", String, callback, ["status", len(ids), id])
      
   #rospy.spin() # simply keeps python from exiting until this node is stopped
   
   rospy.spin()

def call_once(ids):
   create_node()

   for id in ids:
      try:
         rospy.loginfo(rospy.wait_for_message("cf%s/log1" %id, GenericLogData, timeout=2).values)
      except rospy.exceptions.ROSException:
         pass

if __name__ == '__main__':
   ids = sys.argv[1:]
   #call_once(ids)
   listener(ids)
    
    
   #rosbag record -a /cf1/log1
   #rostopic echo /cf1/log1 -b bagFileName.bag -p > fileName.csv
   
   
 #/home/adam/Documents/Packages/crazyswarm/ros_ws/src/crazyswarm/msg/GenericLogData.msg