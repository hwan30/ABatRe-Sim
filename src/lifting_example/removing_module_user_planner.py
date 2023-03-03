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

plan_status=0
def callback1(msg):
       global plan_status
       plan_status=msg.data

rospy.Subscriber("objection_position_pose",Pose,callback)
rospy.Subscriber("plan_control_status", std_msgs.msg.Int8, callback1)

target_pub=rospy.Publisher("planning_target",Pose,queue_size=10)
#initialize
moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('move_group_grasp', anonymous=True)

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

rospy.sleep(2)

arm_group = moveit_commander.move_group.MoveGroupCommander("manipulator")
arm_group.allow_replanning(True)


#set up collision objects

#set up collision objects
p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.5+0.25
p.pose.position.y = 1-0.8
p.pose.position.z = 0.3-0.55
scene.add_box("floor", p, (0.8, 1.5, 0.1))
print('floor collision object finished')

p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.1+0.25
p.pose.position.y = 1-0.8
p.pose.position.z = 0.35-0.55
scene.add_box("Left_base_plate", p, (0.02, 1.5, 0.2))
print('Left base_plate collision object finished')

p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.5+0.25
p.pose.position.y = 1-0.8
p.pose.position.z = 0-0.55
scene.add_box("floor", p, (2, 2, 0.1))
print('ground collision object finished')


p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.5+0.25
p.pose.position.y = 1-0.8
p.pose.position.z = 0.35-0.55
scene.add_box("module_block", p, (0.8, 1.4, 0.18))
print('battery collision object finished')


while True:

        #spawn and pick up
        pose_target = geometry_msgs.msg.Pose()


        pose_target.orientation.x = -0.5
        pose_target.orientation.y = 0.5
        pose_target.orientation.z = 0.5
        pose_target.orientation.w = 0.5

        #previously using moveit planning the tf handles, world and robot frame transfrom, user script does not, add offset for worldframe
        pose_target.position.x=x#-0.25
        pose_target.position.y=y#+0.8
        pose_target.position.z=z+0.02#+0.55
        target_pub.publish(pose_target)
        print("published to move module ")
        print("waiting for finish ")
        rospy.sleep(15)
        
        if plan_status==-1:
                print("user planner failed, use moveit ")
               #user planner failed, use second resort, moveit planner
                arm_group.set_pose_target(pose_target)
                plan = arm_group.go()
                print("moveit finished  ")
        
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
