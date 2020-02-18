
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <sstream>
#include "std_msgs/String.h"
#include <stdio.h>
#include <ros/callback_queue.h>
#include <stdlib.h>
#include <chrono>
#include <sys/types.h>
#include <sys/stat.h>
#include <fstream>
#include <vector>
#include <rrtstar_msgs/rrtStarSRV.h>
#include <rrtstar_msgs/Region.h>
#include <geometry_msgs/Vector3.h>
//#include <cmat/cmat.h>
#include <simRobot_msgs/simulateRobotSRV.h>
#include <simRobot_msgs/sim_robot.h>
#include <simRobot_msgs/transformation.h>


#include"youbotCallback.hpp"
#include "youbotcmd_msgs/ControlYoubot.h"



typedef vector < vector <float> > vect2;



int main(int argc, char** argv) {
	ros::init(argc, argv, "api_hri_pitt_youbot");
	ros::NodeHandle nh;

	//	ros::Publisher pubPathReg=nh.advertise<std_msgs::String>("pathPlannerRegions",1); // later change as a srv
	//	ros::Publisher pubPathObs=nh.advertise<std_msgs::String>("pathPlannerObstacles",1);

	//! Definition control interface parameters:
	ros::Publisher pub_ctrl_cmnd=nh.advertise<youbotcmd_msgs::ControlYoubot>("youbot_control_command",80);
	ros::Publisher pub_ctrl_error=nh.advertise<std_msgs::String>("hri_control_error_check",80);
	const int NO_ARMS=2;
	const int No_Sphere=0;		//! number of sphere in workspace
	const int No_Cylinder=2;	//! number of Cylinder in workspace
	const int No_Cone=0;		//! number of Cone in workspace
	const int No_Plane=1;		//! number of Plane in workspace
	const int No_Unknown=0;		//! number of Unknown Objects in workspace

	int NO_ArmState=1;	//left,right,bimanual
	int NO_ctrlCmdType=5;	//SigleArm,biManual,Gripper, Stop,HoldingMode
	int ctrlCmdTypeGoalReachedCounter=0, armStateGoalReachedCounter=0;
	int responsible_arm=0; //! the arm which should grasp the object (0,1,2)
	int num_joint=7;

	const int No_Objects=No_Sphere+No_Cylinder+No_Cone+No_Plane;

	std_msgs::String msg_ctrl_err;
	bool control_error_flag=true, control_error_stop_flag=true; //DEL, first
	std_msgs::String msg_ctrl_cmnd[NO_ArmState] ;
	bool robot_send_control_command_flag[NO_ArmState];
	int control_count[NO_ArmState], control_goal_count[NO_ArmState][NO_ctrlCmdType];

	for (int i1=0;i1<NO_ArmState;i1++)
	{
		robot_send_control_command_flag[i1]=false;// when true robot sends a command
		control_count[i1]=0;
		for (int j1=0;j1<NO_ctrlCmdType;j1++)
			control_goal_count[i1][j1]=0;
	}
	int rob_goal_reach_flag_counter=0;
	std::stringstream ss_ctrl_cmnd;
	int noParamCtrlInit=2;//Initial Control Parameters
	int noParamCtrl=6;//Execution Control Parameters, 		[1st: set/get, , 2-4: Actions Parameters,]
	float paramCtrlInit [noParamCtrlInit];// Initial Parameters of Controller
	float paramCtrl[noParamCtrl];// first input: set Data: 1, or get Data:0
	int max_time_rob_reach_goal=50;//sec

// rrt* Service:
	ros::ServiceClient rrtStar_client = nh.serviceClient<rrtstar_msgs::rrtStarSRV>("rrtStarService");
	rrtstar_msgs::rrtStarSRV rrtSRV;
	rrtstar_msgs::Region obstacle;
	geometry_msgs::Vector3 pints;

// robot simulator service:
	ros::ServiceClient simRobot_client = nh.serviceClient<simRobot_msgs::simulateRobotSRV>("robotSimulator_service");
	simRobot_msgs::simulateRobotSRV simRobot_srv;
	simRobot_msgs::transformation simRobot_pose;



	youbotcmd_msgs::ControlYoubot control_msg[NO_ArmState];

	float InitPose[6],obstacleBoundingBox[6],pathPlanningPoseGoal[6], graspingPoseGoal[6];
	float GoalOrientation[3],goalRegion[3];
	for (int i=0;i<6;i++)
	{
		InitPose[i]=0.0;
		obstacleBoundingBox[i]=0.0;
		pathPlanningPoseGoal[i]=0.0;
		graspingPoseGoal[i]=0.0;
	}

	GoalOrientation[0]=0.0;GoalOrientation[1]=0.0;GoalOrientation[2]=0.0;
	goalRegion[0]=0.04;goalRegion[1]=0.04;goalRegion[2]=0.04;

	//	vector <float **> pathVector;
	vector<float> pathPoint(6,0.0); //! each points of path
	vect2 Path;
	vector < vect2  > pathVector;	//! we can compute for each arm state the path (left and right or biman)
	vector<bool> pathPointsFlag;	//! save each point in the path if it is done (true) or it is not done (false)
	vector < vector<bool> > pathPointsFlagVector; //! save pathPointsFlag for all arm states in a vector
	vector <int> pathNO_ArmStateVector; //! save for each element in pathvector, which arm state it is.

	vector <int> pathSizeVector; //! for each arm state save what is the path size.
	pathVector.resize(NO_ArmState);
	pathPointsFlagVector.resize(NO_ArmState);
	pathSizeVector.resize(NO_ArmState);

	bool pathCheck[NO_ArmState];
	pathCheck[0]=false;pathCheck[1]=false;pathCheck[2]=false;
	int pathPointNumber[NO_ArmState];	//! save the number of point we are in the path
	pathPointNumber[0]=0;pathPointNumber[1]=0;pathPointNumber[2]=0;
	string controlState="armControl";

	int pathNO_ArmState;

	//float **Path;
	int pathSize=0;			//! rrtStar output path size
	//Path=new float*[6]; 	//! Yaw pitch roll, x y z

	int pathPointsFlagCounter=0;
	int pathDoneCounterArms=0;
	int No_activeArmState=0;

	//	bool arms[NO_ArmState];
	//	arms[0]=false;arms[1]=false;arms[2]=false;


	int aNumber;

	int ros_freq=80;//hz
	ros::Rate loop_rate(ros_freq);//
	long int count=0;
	usleep(1e2);

	youbotCallback robot_call(No_Sphere,No_Cylinder,No_Plane,No_Cone,No_Unknown);

	bool ObjectRecognition=false;			//! if this flag is true we should check environment and get information from it;
	bool pathPlanningFlag=true;	//! if it is true we should do path planning
	bool NopathPlanningFlag=false;	//! path planning is not necessary.
	bool simulationFlag=false;
	bool controllerFlag= false;		//! if it is true we should do path planning
	int grasping_obj_id=0;

	controllerFlag=true;
	while (ros::ok()) {



		robot_call.FailureCheck();


		if(count==0)
			{usleep(0.5e6);}

		loop_rate.sleep();
		ros::spinOnce();
		count++;
	}
	return 0;
}


