#! /usr/bin/env python
#coding=utf-8
import sys
import rospy
import moveit_commander
import geometry_msgs
import roslaunch
from std_msgs.msg import Int32



from std_msgs.msg import Bool
from gazebo_msgs.srv import DeleteModel, SpawnModel ,GetModelState
from geometry_msgs.msg import *
from copy import deepcopy

reset=0
pub=rospy.Publisher('reset_flag',Bool,queue_size=10)
setup=0

x=0
y=0
z=0
def callback(msg):
    global x,y,z
    x=msg.position.x
    y=msg.position.y
    z=msg.position.z
#    print("inside callback:",z)

p_stage=1

def callback1(msg):
    global p_stage
    p_stage=msg.data
    #print('p_stage is',p_stage)

rospy.Subscriber("objection_position_pose",Pose,callback)
rospy.Subscriber("stage",Int32,callback1)


#initialize 
moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('move_group_grasp', anonymous=True)



pub.publish(reset)

cartesian = rospy.get_param('~cartesian', True)

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

rospy.sleep(2)

arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
arm_group.allow_replanning(True)
#arm.set_planning_time(20)


#set up collision objects
p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.5
p.pose.position.y = 1
p.pose.position.z = 0.3
scene.add_box("floor", p, (0.8, 1.5, 0.1))
print('floor collision object finished')

p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.1
p.pose.position.y = 1
p.pose.position.z = 0.35
scene.add_box("Left_base_plate", p, (0.02, 1.5, 0.2))
print('Left_base_plate collision object finished')

p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.5
p.pose.position.y = 1
p.pose.position.z = 0
scene.add_box("floor", p, (2, 2, 0.1))
print('ground collision object finished')

#replace static object with movable objects
rospy.wait_for_service("gazebo/spawn_sdf_model")
rospy.wait_for_service("gazebo/delete_model")

model_coordinates=rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	
resp_coordinates = model_coordinates('HW_tap_bolt_sc_f_0', '') 

#duplicate
resp_coordinates_1 = model_coordinates('HW_tap_bolt_sc_f_1', '') 
resp_coordinates_2 = model_coordinates('HW_tap_bolt_sc_f_2', '') 
resp_coordinates_3 = model_coordinates('HW_tap_bolt_sc_f_3', '') 
resp_coordinates_4 = model_coordinates('HW_tap_bolt_sc_f_4', '') 
resp_coordinates_5 = model_coordinates('HW_tap_bolt_sc_f_5', '') 

f=open('/home/wanghu26/battery_ws/src/battery_model/model_editor_models/HW_tap_bolt_sc/model.sdf','r')
sdff=f.read()

spawn_model=rospy.ServiceProxy("gazebo/spawn_sdf_model",SpawnModel)
spawn_model('new_bolt0',sdff,"",resp_coordinates.pose,"world")

#duplicate 
spawn_model('new_bolt1',sdff,"",resp_coordinates_1.pose,"world")
spawn_model('new_bolt2',sdff,"",resp_coordinates_2.pose,"world")
spawn_model('new_bolt3',sdff,"",resp_coordinates_3.pose,"world")
spawn_model('new_bolt4',sdff,"",resp_coordinates_4.pose,"world")
spawn_model('new_bolt5',sdff,"",resp_coordinates_5.pose,"world")


#rospy.sleep(1)
delete_model=rospy.ServiceProxy("gazebo/delete_model",DeleteModel)
delete_model('HW_tap_bolt_sc_f_0')
print('HW_tap_bolt_sc_f_0 delete')
#duplicate 
delete_model('HW_tap_bolt_sc_f_1')
print('HW_tap_bolt_sc_f_1 delete')
delete_model("HW_tap_bolt_sc_f_2")
print('HW_tap_bolt_sc_f_2 delete')
delete_model("HW_tap_bolt_sc_f_3")
print('HW_tap_bolt_sc_f_3 delete')
delete_model("HW_tap_bolt_sc_f_4")
print('HW_tap_bolt_sc_f_4 delete')
delete_model("HW_tap_bolt_sc_f_5")
print('HW_tap_bolt_sc_f_5 delete')


while True:
    if p_stage==1:
    #remove bolt
        arm_group.set_named_target("home_j")
        plan = arm_group.go()

        rospy.sleep(2)


        end_effector_link = arm_group.get_end_effector_link()

        start_pose = arm_group.get_current_pose(end_effector_link).pose


        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 0.5
        pose_target.orientation.x = -0.5
        pose_target.orientation.y = 0.5
        pose_target.orientation.z = -0.5
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z+0.27
        #print("x to be executed:",pose_target.position.x)
        #print("y to be executed:",pose_target.position.y)
        #print("z to be executed:",pose_target.position.z)


        reset=0


        arm_group.set_pose_target(pose_target)
        plan = arm_group.go()

        hand_group.set_named_target("close")
        plan = hand_group.go()

        #unscrewing
        angles=arm_group.get_current_joint_values()
        print(angles)
        print(angles[5])
        angles[5]=angles[5]+3.1415
        arm_group.set_joint_value_target(angles)
        plan=arm_group.go()
        print(angles)
        print(angles[5])

        #lift
        pose_target.position.z = 1.1
        arm_group.set_pose_target(pose_target)
        plan = arm_group.go()

        #add a home position to help with planning, some the further bolt wil result in planning fail
        #arm_group.set_named_target("home_j")
        #plan = arm_group.go()


        pose_target.position.x = 0
        pose_target.position.y = 0.2
        arm_group.set_pose_target(pose_target)
        plan = arm_group.go()

        hand_group.set_named_target("open")
        plan = hand_group.go()


        rospy.sleep(2)
        pub.publish(reset)



        rospy.sleep(2)
        reset=1
        pub.publish(reset)

    if p_stage==2:
        if setup==0:
            print("----------------enter stage 2 ----------------step up movable objects")
            #spawn and delete
            rospy.wait_for_service("gazebo/spawn_sdf_model")
            rospy.wait_for_service("gazebo/delete_model")

            model_coordinates=rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	
            resp_coordinates = model_coordinates('HW_HV_cable_sc_f', '') 

            #duplicate
            resp_coordinates_1 = model_coordinates('HW_HV_cable_sc_f_1', '') 



            f=open('/home/wanghu26/battery_ws/src/battery_model/model_editor_models/HW_HV_cable_sc/model.sdf','r')
            sdff=f.read()

            spawn_model=rospy.ServiceProxy("gazebo/spawn_sdf_model",SpawnModel)
            spawn_model('new_cable0',sdff,"",resp_coordinates.pose,"world")

            #duplicate 
            spawn_model('new_cable1',sdff,"",resp_coordinates_1.pose,"world")


            #rospy.sleep(1)
            delete_model=rospy.ServiceProxy("gazebo/delete_model",DeleteModel)
            delete_model('HW_HV_cable_sc_f')
            print('HW_tap_bolt_sc_f_0 delete')
            #duplicate 
            delete_model('HW_HV_cable_sc_f_1')
            print('HW_tap_bolt_sc_f_1 delete')
        
            global setup
            setup=1
        else:

            print("-------------still in stage 2 -----------")
            arm_group.set_named_target("home_j")
            plan = arm_group.go()


            end_effector_link = arm_group.get_end_effector_link()

            start_pose = arm_group.get_current_pose(end_effector_link).pose


            pose_target = geometry_msgs.msg.Pose()
            pose_target.orientation.w = 0.5
            pose_target.orientation.x = -0.5
            pose_target.orientation.y = 0.5
            pose_target.orientation.z = -0.5
            pose_target.position.x = x
            pose_target.position.y = y
            pose_target.position.z = z+0.28
            #print("x to be executed:",pose_target.position.x)
            #print("y to be executed:",pose_target.position.y)
            #print("z to be executed:",pose_target.position.z)



            reset=0


            arm_group.set_pose_target(pose_target)
            plan = arm_group.go()

            hand_group.set_named_target("close")
            plan = hand_group.go()


            #lift
            pose_target.position.z = 0.9
            arm_group.set_pose_target(pose_target)
            plan = arm_group.go()

            pose_target.position.x = 0
            arm_group.set_pose_target(pose_target)
            plan = arm_group.go()

            hand_group.set_named_target("open")
            plan = hand_group.go()


            rospy.sleep(2)
            pub.publish(reset)



            rospy.sleep(2)
            reset=1
            pub.publish(reset)
    
    if p_stage==3:
        print('------enter stage 3 -------')
          

        #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)
        #launch_file=['ur10_notebook','ur10_remove.launch']
        #roslaunch_file=roslaunch.rlutil.resolve_launch_arguments(launch_file)
        #parent = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_file])
        #parent.start()
        break


moveit_commander.roscpp_initializer.roscpp_shutdown()
