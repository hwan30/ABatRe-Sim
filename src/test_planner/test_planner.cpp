#include<string.h>
#include<iostream>
#include<boost/shared_ptr.hpp>
#include<ctime>

#include<ros/ros.h>
#include<ros/subscriber.h>
#include<ros/publisher.h>

#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>

#include<std_msgs/Bool.h>
#include<std_msgs/Int8.h>
#include<geometry_msgs/Pose.h>
#include<moveit_msgs/RobotState.h>
#include<moveit_msgs/RobotTrajectory.h>

#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/robot_model/robot_model.h>
#include<moveit/planning_scene/planning_scene.h>
#include<moveit/planning_scene_monitor/planning_scene_monitor.h>

#include<moveit/planning_interface/planning_interface.h>
#include<moveit/move_group_interface/move_group.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
#include<moveit/collision_detection/collision_common.h>

#include<moveit/trajectory_processing/iterative_spline_parameterization.h>
#include<moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>


// Important informations
static const std::string PLANNING_GROUP="manipulator";
const int DOF=6;

const int maxIKTimes=50;
// Parameters for RRT
const double PI=3.14159;
const int maxIteration=2000;
const int maxSamplePerPeriod=1;
const double maxStep=0.4,minStep=0.1;
double rewireRange=0.5;
double maxRange=3*PI,minRange=-3*PI;
const int checkNum=3;   //number of checking for collision avoidance

// global Variables
geometry_msgs::Pose poseTarget;
bool startPlan=false;
ros::V_string jointNames;

std::vector<double> eigen2std(Eigen::Matrix<double,DOF,1> input)
{
    std::vector<double> output;
    for(int i=0;i<DOF;i++)
    {
        output.push_back(input(i));
    }
    return output;
}

class RRTNode
{
private:
public:
    Eigen::Matrix<double,DOF,1> q;
    double dist_to_root;
    int prevNodeID;
    RRTNode(Eigen::Matrix<double,DOF,1> q_,double distToRoot,int prevNodeID_)
    {
        q=q_;
        dist_to_root=distToRoot;
        prevNodeID=prevNodeID_;
    }
    RRTNode(std::vector<double> q_vec,double distToRoot,int prevNodeID_)
    {
        for(int i=0;i<DOF;i++)q(i)=q_vec[i];
        dist_to_root=distToRoot;
        prevNodeID=prevNodeID_;
    }
    Eigen::Matrix<double,DOF,1> CalcDisplacement(RRTNode otherNode)
    {
        Eigen::Matrix<double,DOF,1> result;
        for(int i=0;i<DOF;i++)
        {
            result(i)=otherNode.q(i)-q(i);
            while(result(i)<=-PI || result(i)>PI)
            {
                if(result(i)>PI)result(i)-=2*PI;
                else if(result(i)<=-PI)result(i)+=2*PI;
            }
        }
        return result;
    }
    ~RRTNode(){};
};

RRTNode Sample()
{
    // unbiased
    Eigen::Matrix<double,DOF,1> q_new=Eigen::Matrix<double,DOF,1>::Random()*maxRange;
    return RRTNode(q_new,1000000,-10);
}

void Extend(std::vector<RRTNode> &tree, RRTNode newNode, planning_scene::PlanningScenePtr planningScenePtr)
{
    // initialize for collision checking
    moveit_msgs::RobotState desiredRobotState;
    desiredRobotState.joint_state.name=jointNames;
    // desiredRobotState.joint_state.position=std::vector<double>{1.57*0,0,0,0,0,0};
    // ROS_INFO(planningScenePtr->isStateValid(desiredRobotState)?"valid":"invalid");
    
    
    // extend the tree, then rewire
    // find the nearest node on the tree
    
    int nearID=-1;
    double nearDist=1000000;
    Eigen::Matrix<double,DOF,1> nearDisplacement;
    for(int i=0;i<tree.size();i++)
    {
        Eigen::Matrix<double,DOF,1> tempDisplacement=tree[i].CalcDisplacement(newNode);
        if(tempDisplacement.norm()<nearDist)
        {
            nearID=i;
            nearDist=tempDisplacement.norm();
            nearDisplacement=tempDisplacement;
        }
    }
    //for debug
    if(nearID==-1)std::cout<<"??????\n\n\n";
    // steer and add the new node
    if(nearDist>maxStep)
    {
        nearDisplacement*=(maxStep/nearDist);
        nearDist=maxStep;
    }
    else if(nearDist<minStep)
    {
        return;
    }
    newNode=RRTNode(tree[nearID].q+nearDisplacement,tree[nearID].dist_to_root+nearDist,nearID);

    //collision checking
    bool newNodeValid=true;
    for(int i=0;i<=checkNum;i++)
    {
        Eigen::Matrix<double,DOF,1> q_=(i*newNode.q+(checkNum-i)*tree[nearID].q)/checkNum;
        desiredRobotState.joint_state.position=eigen2std(q_);
        if(!planningScenePtr->isStateValid(desiredRobotState))
        {
            newNodeValid=false;
            break;
        }
    }
    if(!newNodeValid)return;
    
    // extend
    tree.push_back(newNode);
    // rewire
    int i_max=tree.size()-1;
    for(int i=0;i<i_max;i++)
    {
        Eigen::Matrix<double,DOF,1> tempDisplacement=tree[i].CalcDisplacement(tree[i_max]);
        double tempDist=tempDisplacement.norm();
        if(tempDist<rewireRange)
        {
            if(tree[i_max].dist_to_root+tempDist<tree[i].dist_to_root)
            {
                tree[i].dist_to_root=tree[i_max].dist_to_root+tempDist;
                tree[i].prevNodeID=i_max;
            }
        }
    }
}

bool GoalCheck(std::vector<RRTNode> &tree, RRTNode goalNode, planning_scene::PlanningScenePtr planningScenePtr)
{
    // initialize for collision checking
    moveit_msgs::RobotState desiredRobotState;
    desiredRobotState.joint_state.name=jointNames;
    // desiredRobotState.joint_state.position=std::vector<double>{1.57*0,0,0,0,0,0};
    // ROS_INFO(planningScenePtr->isStateValid(desiredRobotState)?"valid":"invalid");
    
    
    //Check + biased sampling
    std::vector<RRTNode> tempTree=tree;
    bool success=false;
    // find the nearest node on the tree
    int nearID=-1;
    double nearDist=1000000;
    Eigen::Matrix<double,DOF,1> nearDisplacement;
    for(int i=0;i<tree.size();i++)
    {
        Eigen::Matrix<double,DOF,1> tempDisplacement=tree[i].CalcDisplacement(goalNode);
        if(tempDisplacement.norm()<nearDist)
        {
            nearID=i;
            nearDist=tempDisplacement.norm();
            nearDisplacement=tempDisplacement;
        }
    }
    // add goalNode to tree if it's close enough
    if(nearDist<maxStep)
    {
        tree.push_back(RRTNode(tree[nearID].q+nearDisplacement,tree[nearID].dist_to_root+nearDist,nearID));
        success=true;
    }
    else
    {
        nearDisplacement*=(maxStep/nearDist);
        nearDist=maxStep;
        RRTNode newNode(tree[nearID].q+nearDisplacement,tree[nearID].dist_to_root+nearDist,nearID);
        //collision checking
        bool newNodeValid=true;
        for(int i=0;i<=checkNum;i++)
        {
            Eigen::Matrix<double,DOF,1> q_=(i*newNode.q+(checkNum-i)*tree[nearID].q)/checkNum;
            desiredRobotState.joint_state.position=eigen2std(q_);
            if(!planningScenePtr->isStateValid(desiredRobotState))
            {
                newNodeValid=false;
                break;
            }
        }
        if(newNodeValid)tree.push_back(newNode);
    }
    return success;
}

void onRecvTarget(const geometry_msgs::PoseConstPtr &Ptr)
{
    poseTarget=*Ptr;
    poseTarget.position.x;//+=0.25;
    poseTarget.position.y;//-=0.8;
    poseTarget.position.z;//-=0.55;
    
    startPlan=true;
}

int main(int argc,char** argv)
{
    srand((unsigned)time(NULL));
    // init
    ros::init(argc,argv,"test_planner");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    ros::AsyncSpinner asyncSpinner(10);
    asyncSpinner.start();

    // set up subscriber and publisher
    ros::Subscriber targetSub=nh.subscribe("/planning_target",10,&onRecvTarget);
    ros::Publisher planPub=nh.advertise<moveit_msgs::RobotTrajectory>("/generated_plan",10);
    ros::Publisher status=nh.advertise<std_msgs::Int8>("/plan_control_status",10); //0:ready to receive command; 1:planning; 2:success; -1:failed
    std_msgs::Int8 PlannerStatus;
    PlannerStatus.data=0;
    status.publish(PlannerStatus);


    // set up robot model
    moveit::planning_interface::MoveGroupInterface moveGroup(PLANNING_GROUP);
    jointNames=moveGroup.getJointNames();
    robot_model::RobotModelConstPtr robotModelPtr=moveGroup.getRobotModel();
    const robot_state::JointModelGroup *jointModelGroup=robotModelPtr->getJointModelGroup(PLANNING_GROUP);
    robot_state::RobotStatePtr robotStatePtr(new robot_state::RobotState(robotModelPtr));

    // collision check variables
    moveit_msgs::RobotState desiredRobotState;
    desiredRobotState.joint_state.name=jointNames;
    
    // trajectory processing variables
    robot_trajectory::RobotTrajectory robotTrajectory(robotModelPtr,PLANNING_GROUP);
    moveit_msgs::RobotTrajectory executableTraj,tempTraj;
    executableTraj.joint_trajectory.joint_names=jointNames;
    trajectory_msgs::JointTrajectoryPoint tempJointTrajPoint;
    trajectory_processing::IterativeSplineParameterization isp;

    while(ros::ok())
    {
        if(startPlan)
        {
            // set up collision checking
            planning_scene_monitor::PlanningSceneMonitor monitor("robot_description");
            monitor.requestPlanningSceneState("get_planning_scene");
            planning_scene::PlanningScenePtr planningScenePtr=monitor.getPlanningScene();
            planningScenePtr->decoupleParent();
            // // check collision example
            // desiredRobotState.joint_state.position=std::vector<double>{1.57*-0.,0,0,0,0,0};
            // ROS_INFO(planningScenePtr->isStateValid(desiredRobotState)?"this state is valid":"this state is invalid");

            ROS_INFO("Target received, planning...");
            std::cout<<"position: "<<poseTarget.position.x<<"\t"<<poseTarget.position.y<<"\t"<<poseTarget.position.z<<"\n";
            std::cout<<"orientation: "<<poseTarget.orientation.w<<"\t"<<poseTarget.orientation.x<<"\t"<<poseTarget.orientation.y<<"\t"<<poseTarget.orientation.z<<"\n";
            PlannerStatus.data=1;
            status.publish(PlannerStatus);
            
            //IK
            std::vector<double> targetJointValue;
            // robotStatePtr->setFromIK(jointModelGroup,poseTarget,1,0.1);
            // robotStatePtr->copyJointGroupPositions(jointModelGroup,targetJointValue);
            // desiredRobotState.joint_state.position=targetJointValue;
            for(int IKTryTime=0;IKTryTime<maxIKTimes;IKTryTime++)
            {
                robotStatePtr->setFromIK(jointModelGroup,poseTarget,5,0.2);
                robotStatePtr->copyJointGroupPositions(jointModelGroup,targetJointValue);
                desiredRobotState.joint_state.position=targetJointValue;
                if(planningScenePtr->isStateValid(desiredRobotState))break;
            }
            if(!planningScenePtr->isStateValid(desiredRobotState))
            {
                ROS_ERROR("Invaild target.");
                startPlan=false;
                PlannerStatus.data=-1;
                status.publish(PlannerStatus);
                continue;
            }
            
            //RRT
            std::vector<RRTNode> tree({});
            RRTNode startNode(moveGroup.getCurrentJointValues(),0,-1);
                std::cout<<"start configuration: "<<startNode.q.transpose()<<"\n";
            tree.push_back(startNode);
            RRTNode goalNode(targetJointValue,100000000,-10);
                std::cout<<"goal configuration: "<<goalNode.q.transpose()<<"\n";
            bool planSucceed=false;
            for(int itr=0;itr<maxIteration;itr++)
            {
                for (int s=0;s<maxSamplePerPeriod;s++)
                {
                    RRTNode newNode=Sample();
                    Extend(tree,newNode,planningScenePtr);
                }
                if(GoalCheck(tree,goalNode,planningScenePtr))
                {
                    planSucceed=true;
                    break;
                }
            }
            if(!planSucceed)
            {
                ROS_WARN("Plan failed... %i nodes in total",tree.size());
                startPlan=false;
                // for(int tree_i=0;tree_i<tree.size();tree_i++)
                // {
                //     std::cout<<tree[tree_i].CalcDisplacement(goalNode).norm()<<'\n';
                //     std::cout<<tree[tree_i].q.transpose()<<'\n';
                //     std::cout<<tree[tree_i].dist_to_root<<'\n';
                //     std::cout<<tree[tree_i].prevNodeID<<'\n'<<'\n';
                // }
                PlannerStatus.data=-1;
                status.publish(PlannerStatus);
                continue;
            }
            else
            {
                ROS_INFO("Plan found, %i nodes in total, executing...",tree.size());
            }

            PlannerStatus.data=2;
            status.publish(PlannerStatus);

            //Trajectory generation
            int currentID=tree.size()-1;
            while(currentID!=-1)
            {
                tempJointTrajPoint.positions=eigen2std(tree[currentID].q);
                tempTraj.joint_trajectory.points.push_back(tempJointTrajPoint);
                // std::cout<<"Node "<<currentID<<": "<<tree[currentID].q.transpose()<<"\n";
                currentID=tree[currentID].prevNodeID;
            }
            int i_max=tempTraj.joint_trajectory.points.size()-1;
            for(int i=0;i<=i_max;i++)
            {
                executableTraj.joint_trajectory.points.push_back(tempTraj.joint_trajectory.points[i_max-i]);
            }
            
            //Trajectory Time Parameterization
            robotStatePtr->update();
            robotTrajectory.setRobotTrajectoryMsg(*robotStatePtr,executableTraj);
            isp.computeTimeStamps(robotTrajectory,0.5,0.5);
            robotTrajectory.getRobotTrajectoryMsg(executableTraj);

            //Publish the plan
            planPub.publish(executableTraj);
            startPlan=false;
            ROS_INFO("Plan finished.");
            
            //Execute
            moveit::planning_interface::MoveGroupInterface::Plan plan_;
            plan_.trajectory_=executableTraj;
            moveGroup.execute(plan_);
            
            //Erase Temp Variables
            executableTraj.joint_trajectory.points.clear();
            tempTraj.joint_trajectory.points.clear();
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
