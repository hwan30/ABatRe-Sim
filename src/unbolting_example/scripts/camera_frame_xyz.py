#! /usr/bin/env python

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import rospy
import time
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32

import tf

global p0,p1,p2
def object_position_pose(p0,p1,p2):

    pub = rospy.Publisher('/cameraview_position_pose',Pose,queue_size=10)
    p = Pose()
    rate = rospy.Rate(5)
    p.position.x = p0
    p.position.y = p1
    p.position.z = p2

    p.orientation.x = 0
    p.orientation.y = 0
    p.orientation.z = 0
    p.orientation.w = 1
    pub.publish(p)
    
    br = tf.TransformBroadcaster()
    br.sendTransform((p0, p1, p2),
                         (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
                         rospy.Time.now(),
                         "object_camera_frame",
                         "camera_link_optical")
    rate.sleep()

def callback_pointcloud(data):

    global p0,p1,p2,u,v
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True,uvs=[(u,v)])
    time.sleep(1)
    print type(gen)
    for p in gen:
      print " The coordinates of the object from camera frame is x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])
    p0=p[0]
    p1=p[1]
    p2=p[2]
    
    object_position_pose(p0,p1,p2)

def callback1(msg):
    global u
    u=msg.data
def callback2(msg):
    global v
    v=msg.data

def main():
    p0=0
    p1=1
    p2=2
    rospy.init_node('pcl_listener', anonymous=True)

    rospy.Subscriber('/u_coordinates', Int32, callback1)
    rospy.Subscriber('/v_coordinates', Int32, callback2)
    
    rospy.Subscriber('/Kinect/depth/points', PointCloud2, callback_pointcloud)


    rospy.spin()

if __name__ == "__main__":
    main()

