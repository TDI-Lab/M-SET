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
   if ENABLE_LOGGING == True:
      print("\nWARNING: LOG OUTPUT FILE NOT FOUND")
   else:
      pass
start = time.time()

status = ["idle", "idle", "idle", "idle"]

count=0
c=0

# Process data retrieved from the ROS subscribers
def callback(data,args):
   global c
   global count
   global status
   topic = args[0]
   ndrones = args[1]
   id = args[2]

   if topic=="status":
      rospy.loginfo(data.data)
      if id != None:
         msg = data.data.split(':')[min(1,len(data.data.split(':'))-1)] 
         status[int(id)-1] = msg
      else:
         for s in status:
            s = data.data
   else:
      rospy.loginfo("%s: Drone %s: Data: %s" % (count, id, data.values))
      file.write("\n")
      file.write(str(",".join((str(time.time()-start),str(count),str(id),str(data.values),"\""+str(status[int(id)-1])+"\""))))
      c+=1

   count = c // ndrones

def create_node():
   try:
      rospy.init_node('listener', anonymous=True)
   except:
      pass

# Set up the ROS subscribers for retrieving logging data
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
      rospy.Subscriber("/cf%s/log1" %id, GenericLogData, callback, ["log1", len(ids), id]) # MAKE SURE RIGHT CHANNEL IS SET
   
      rospy.Subscriber("status_logger", String, callback, ["status", len(ids), id])
   
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
   listener(ids)