#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs

from gazebo_msgs.srv import DeleteModel, SpawnModel ,GetModelState
from geometry_msgs.msg import *
from std_srvs.srv import Empty

#execfile("spawn_delete_module.py")

x=0
y=0
z=0
def callback(msg):
    global x,y,z
    x=msg.position.x
    y=msg.position.y
    z=msg.position.z
#    print("inside callback:",z)

rospy.Subscriber("objection_position_pose",Pose,callback)

#initialize
moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('move_group_grasp', anonymous=True)

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

rospy.sleep(2)

arm_group = moveit_commander.move_group.MoveGroupCommander("manipulator")
arm_group.allow_replanning(True)

#set up collision objects
p = PoseStamped()
#p.header.frame_id = robot.get_planning_frame()
p.header.frame_id = "world"
p.pose.position.x = 0
p.pose.position.y = 0
p.pose.position.z = -0.3
scene.add_box("floor", p, (3, 3, 0.1))
print('floor collision object finished')

p = PoseStamped()
#p.header.frame_id = robot.get_planning_frame()
p.header.frame_id = "world"
p.pose.position.x = 0.1
p.pose.position.y = 1
p.pose.position.z = -0.2
scene.add_box("Left_base_plate", p, (0.05, 3, 0.2))
print('Left_base_plate collision object finished')



while True:
        arm_group.set_named_target("home_j")
        plan = arm_group.go()


        #spawn and pick up
        pose_target = geometry_msgs.msg.Pose()


        pose_target.orientation.x = -0.5
        pose_target.orientation.y = 0.5
        pose_target.orientation.z = 0.5
        pose_target.orientation.w = 0.5

        pose_target.position.x=x
        pose_target.position.y=y
        pose_target.position.z=z+0.02

        arm_group.set_pose_target(pose_target)
        plan = arm_group.go()
        rospy.sleep(2)

        #vaccum on
        rospy.wait_for_service("/ur5/vacuum_gripper4/on")
        try:
	        turn_on=rospy.ServiceProxy('/ur5/vacuum_gripper4/on',Empty)
	        resp=turn_on()
	        print(resp)
        except rospy.ServiceException, e:
                print "Service call failed: %s" % e

        rospy.sleep(2)


        pose_target.position.z=z+0.5
        arm_group.set_pose_target(pose_target)
        plan = arm_group.go()

        #arm_group.set_named_target("home_j")
        #plan = arm_group.go()


        #move away
        #pose_target.position.x=0.06
        #pose_target.position.y=-0.34
        #pose_target.position.z=0.34
        #pose_target.position.x=0.09
        #pose_target.position.y=-0.8
        #pose_target.position.z=0.034

        #arm_group.set_pose_target(pose_target)
        #plan = arm_group.go()

        angles=arm_group.get_current_joint_values()
        print(angles)
        print(angles[0])
        angles[0]=angles[0]+1.56

        rospy.sleep(2)

        arm_group.set_joint_value_target(angles)
        plan=arm_group.go()
        print(angles)
        print(angles[0])





        rospy.wait_for_service("/ur5/vacuum_gripper4/off")

        try:
	        turn_off=rospy.ServiceProxy('/ur5/vacuum_gripper4/off',Empty)
	        resp=turn_off()
	        print(resp)
        except rospy.ServiceException, e:
                print "Service call failed: %s" % e


        rospy.sleep(4)

moveit_commander.roscpp_initializer.roscpp_shutdown()
