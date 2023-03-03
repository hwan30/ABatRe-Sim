#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose
def object_position_pose(t,o):
    pub = rospy.Publisher('/objection_position_pose',Pose,queue_size=10)
    p = Pose()
    rate = rospy.Rate(5)
    p.position.x = t[0]
    p.position.y = t[1]
    p.position.z = t[2]

    p.orientation.x = o[0]
    p.orientation.y = o[1]
    p.orientation.z = o[2]
    p.orientation.w = o[3]
    pub.publish(p)
    rate.sleep()

if __name__ == '__main__':
   rospy.init_node('tf_listener',anonymous=True)
   listener = tf.TransformListener() 
   rate = rospy.Rate(10.0)
   while not rospy.is_shutdown():
       try:
#           listener.waitForTransform("/world","/cameraview_position_pose",rospy.Time(0),rospy.Duration(5))
           (trans,rot) = listener.lookupTransform("/world", "/object_camera_frame", rospy.Time(0))
           print("trans:")
           print(trans)
           print("rot:")
           print(rot)
           object_position_pose(trans,rot)
           print("The coordinates of the object in the world frame is x : %.5f  y: %.5f  z: %.5f" %(trans[0],trans[1],trans[2]))
       except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
           continue
            
        

       rate.sleep()
