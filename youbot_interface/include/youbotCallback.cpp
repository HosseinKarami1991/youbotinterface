
#include "youbotCallback.hpp"
#include <math.h>       /* sin */

#define PI 3.14159265
youbotCallback::youbotCallback(int nosphere,int nocylinder,int noplane,int nocone,int nounknown):NoSphere(nosphere),NoCylinder(nocylinder),NoPlane(noplane),NoCone(nocone),NoUnknown(nounknown){

//	RANSAC parameter Initialization
	NoObjects=NoSphere+NoCylinder+NoPlane;
	NoObstacles=NoSphere+NoCylinder;
	NoCorrectRecognizedObject=0;


EstimatedNoSphere=0;
EstimatedNoCylinder=0;
EstimatedNoPlane=0;
EstimatedNoCone=0;
EstimatedNoUnknown=0;
num_joint=5;

	objectFeatureBox=new float* [NoSphere+NoCylinder];
	objectFeatureBall=new float* [NoSphere+NoCylinder];
	goalID=100;

	for (int i=0;i<NoSphere+NoCylinder;i++){
		objectFeatureBox[i]=new float [6]; 	//! 6: 1-3: center ; 4-6: size
		objectFeatureBall[i]=new float [4];	//! center: x,y,z,radius
	}

	obstacleSafetyFactor=1.0;//1.5, 2.5
	obstacleSafetyFactorX=1.1;
	obstacleSafetyFactorY=3.5;
	obstacleSafetyFactorZ=2.0;

	for (int i=0;i<NoSphere+NoCylinder;i++)
		for (int j=0;j<6;j++)
			objectFeatureBox[i][j]=0;

	for (int i=0;i<NoSphere+NoCylinder;i++)
		for (int j=0;j<4;j++)
			objectFeatureBall[i][j]=0;
//	Path planner initialization:
	for(int i=0;i<6;i++) {
		regionGoal[0][i]=0;
		regionGoal[1][i]=0;
		regionGoal[2][i]=0;
		regionOperating[i]=0;
		initPosLeftArm[i]=0;
		initPosRightArm[i]=0;
		initPos[0][i]=0.0;
		initPos[1][i]=0.0;
		initPos[2][i]=0.0;
		tTo1_arr[i]=0.0;
		tTo2_arr[i]=0.0;
		ReachingPoint[0][i]=0.0;
		ReachingPoint[1][i]=0.0;
		ReachingPoint[2][i]=0.0;
		wTo_BiMan[i]=0.0;
		}

	for (int i=0;i<5;i++){
		init_q_[0][i]=0.0;
	
	}


//****************************
//	0:
//	cartPos right 1.5 0.0 3.1 0.75 -0.384 0.0
//	1:
//	cartPos right 1.5 0.0 3.1 0.75 -0.384 -0.13
//	2:
//	cartPos right 1.5 0.0 3.1 0.75 -0.384 -0.05
//	3:
//	cartPos left 0.0 -1.5 1.5 0.75 0.08 0.195
//	4:
//	cartPos right 3.092 -1.484 1.605 0.75 -0.04 0.21
//	5:
//	cartPos left 0.0 -1.5 1.5 0.75 0.01 0.195
//	6:
//	cartPos right 3.092 -1.484 1.605 0.7 -0.15 0.21
//	7:
//	cartPos left -1.5  0.00  -3.116 0.75 0.15 0.2
//	8:
//	cartPos left -1.5  0.00  -3.116 0.75 0.25 -0.11
//	----------------
//	initPos:
//	9:
//	jointPos right 0.240 -1.156 0.900 1.979 -0.407 0.911 2.135
//	cartPos right 1.568  -0.005  3.112  0.570  -0.185  0.091
//	cartPos right 1.568  -0.005  3.112  0.5  -0.43  -0.02
//	10:
//	jointPos left -0.111 -1.107 -1.067 1.913 0.472 1.018 -2.241
//	cartPos left -1.5  0.0  -3.116  0.578  0.191  0.118
//	cartPos left -1.5  0.0  -3.116  0.5  0.43  -0.02
//  --------------
	//	11: biman wTg
	//	0.0 0.0 0.0 0.7 -0.02 0.23
//	12: biman wTo
//	0.0 0.0 0.0 0.75 -0.02 0.20

	ReachingPointsVector[0][0]=1.5;		ReachingPointsVector[0][1]=0.0;		ReachingPointsVector[0][2]=3.1;
	ReachingPointsVector[0][3]=0.75;	ReachingPointsVector[0][4]=-0.384;	ReachingPointsVector[0][5]=0.0+0.15;

	ReachingPointsVector[1][0]=1.5;		ReachingPointsVector[1][1]=0.0;		ReachingPointsVector[1][2]=3.1;
	ReachingPointsVector[1][3]=0.75;	ReachingPointsVector[1][4]=-0.384;	ReachingPointsVector[1][5]=-0.13+0.15;

	ReachingPointsVector[2][0]=1.5;		ReachingPointsVector[2][1]=0.0;		ReachingPointsVector[2][2]=3.1;
	ReachingPointsVector[2][3]=0.75;	ReachingPointsVector[2][4]=-0.384;	ReachingPointsVector[2][5]=-0.05+0.15;

	ReachingPointsVector[3][0]=0.0;		ReachingPointsVector[3][1]=-1.5;	ReachingPointsVector[3][2]=1.5;
	ReachingPointsVector[3][3]=0.75;	ReachingPointsVector[3][4]=0.08;	ReachingPointsVector[3][5]=0.195+0.15;

	ReachingPointsVector[4][0]=3.092;	ReachingPointsVector[4][1]=-1.484;	ReachingPointsVector[4][2]= 1.605;
	ReachingPointsVector[4][3]=0.75;	ReachingPointsVector[4][4]=-0.04;	ReachingPointsVector[4][5]=0.21+0.15;

	ReachingPointsVector[5][0]=0.0;		ReachingPointsVector[5][1]=-1.5;	ReachingPointsVector[5][2]= 1.5;
	ReachingPointsVector[5][3]=0.75;	ReachingPointsVector[5][4]=0.01;	ReachingPointsVector[5][5]=0.195+0.15;

	ReachingPointsVector[6][0]= 3.09;	ReachingPointsVector[6][1]=-1.48;	ReachingPointsVector[6][2]= 1.605;
	ReachingPointsVector[6][3]=0.7;		ReachingPointsVector[6][4]=-0.15;	ReachingPointsVector[6][5]=0.21+0.15;

	ReachingPointsVector[7][0]=-1.5;	ReachingPointsVector[7][1]=0.00;	ReachingPointsVector[7][2]=-3.116;
	ReachingPointsVector[7][3]=0.75;	ReachingPointsVector[7][4]=0.15;		ReachingPointsVector[7][5]=0.20+0.15;

	ReachingPointsVector[8][0]=-1.5;	ReachingPointsVector[8][1]=0.0;		ReachingPointsVector[8][2]= 3.116;
	ReachingPointsVector[8][3]=0.75;	ReachingPointsVector[8][4]=0.25;	ReachingPointsVector[8][5]=-0.11+0.15;

	ReachingPointsVector[9][0]=1.568;	ReachingPointsVector[9][1]=-0.005;	ReachingPointsVector[9][2]= 3.112;
	ReachingPointsVector[9][3]=0.6;		ReachingPointsVector[9][4]=-0.20;	ReachingPointsVector[9][5]=-0.02+0.15;

	ReachingPointsVector[10][0]=-1.5;	ReachingPointsVector[10][1]=0.0;	ReachingPointsVector[10][2]= -3.116;
	ReachingPointsVector[10][3]=0.6;	ReachingPointsVector[10][4]=0.20;	ReachingPointsVector[10][5]=-0.02+0.15;

	ReachingPointsVector[11][0]=0.0;	ReachingPointsVector[11][1]=0.0;	ReachingPointsVector[11][2]= 0.0;
	ReachingPointsVector[11][3]=0.70;	ReachingPointsVector[11][4]= -0.02;	ReachingPointsVector[11][5]=0.23+0.15;

	ReachingPointsVector[12][0]=0.0;	ReachingPointsVector[12][1]=0.0;	ReachingPointsVector[12][2]= 0.0;
	ReachingPointsVector[12][3]=0.75;	ReachingPointsVector[12][4]= -0.02;	ReachingPointsVector[12][5]=0.20+0.15;


//*****************************

	regionOperating[0]=0.45;	regionOperating[1]=-0.01;		regionOperating[2]=0.045;	//! center
	regionOperating[3]=0.36;	regionOperating[4]=0.86+0.4;		regionOperating[5]=0.31;		//! size

	perception_regionOperating[0]=0.45;	perception_regionOperating[1]=-0.01;	perception_regionOperating[2]=0.045;	//! center
	perception_regionOperating[3]=0.36;	perception_regionOperating[4]=0.86;		perception_regionOperating[5]=0.31;		//! size



	regionGoal[0][0]=0.65;	regionGoal[0][1]=0.15;	regionGoal[0][2]= 0.19;	//! center
	regionGoal[0][3]=0.03;	regionGoal[0][4]=0.03;	regionGoal[0][5]=0.03;			//! size

	regionGoal[1][0]=0.78;	regionGoal[1][1]=-0.20;	regionGoal[1][2]= -0.19;	//! center
	regionGoal[1][3]=0.03;	regionGoal[1][4]=0.03;	regionGoal[1][5]=0.03;			//! size

	regionGoal[2][0]=0.84;	regionGoal[2][1]=-0.15;	regionGoal[2][2]= 0.24;//0.27	//! center
	regionGoal[2][3]=0.03;	regionGoal[2][4]=0.03;	regionGoal[2][5]=0.03;			//! size

	orientationGoal[0][0]=3.14; orientationGoal[0][1]=0.0; orientationGoal[0][2]=-3.14; // left:     yaw, pitch,roll
	orientationGoal[1][0]=3.14; orientationGoal[1][1]=0.0; orientationGoal[1][2]=-3.14; // right:    yaw, pitch,roll
	orientationGoal[2][0]=0.0; orientationGoal[2][1]=-0.65; orientationGoal[2][2]=0.0; // Bimanual: yaw, pitch,roll

//	controller parameters initialization:
	control_ack_flag[0]			=true;
	control_ack_flag[1]			=true;
	control_ack_flag[2]			=true;
	hri_control_goal_flag[0]	=true;
	hri_control_goal_flag[1]	=true;
	hri_control_goal_flag[2]	=true;
	rob_goal_reach_flag[0]		=true;
	rob_goal_reach_flag[1]		=true;
	rob_goal_reach_flag[2]		=true;
	obj_call_back_flag			=true;

	control_goal_reach_ack[0][0]=false;control_goal_reach_ack[1][0]=false;control_goal_reach_ack[2][0]=false;
	control_goal_reach_ack[0][1]=false;control_goal_reach_ack[1][1]=false;control_goal_reach_ack[2][1]=false;
	control_goal_reach_ack[0][2]=false;control_goal_reach_ack[1][2]=false;control_goal_reach_ack[2][2]=false;
	control_goal_reach_ack[0][3]=false;control_goal_reach_ack[1][3]=false;control_goal_reach_ack[2][3]=false;
	control_goal_reach_ack[0][4]=false;control_goal_reach_ack[1][4]=false;control_goal_reach_ack[2][4]=false;

	controlCmndSent_goalNotReached[0][0]=false;controlCmndSent_goalNotReached[1][0]=false;controlCmndSent_goalNotReached[2][0]=false;
	controlCmndSent_goalNotReached[0][1]=false;controlCmndSent_goalNotReached[1][1]=false;controlCmndSent_goalNotReached[2][1]=false;
	controlCmndSent_goalNotReached[0][2]=false;controlCmndSent_goalNotReached[1][2]=false;controlCmndSent_goalNotReached[2][2]=false;
	controlCmndSent_goalNotReached[0][3]=false;controlCmndSent_goalNotReached[1][3]=false;controlCmndSent_goalNotReached[2][3]=false;
	controlCmndSent_goalNotReached[0][4]=false;controlCmndSent_goalNotReached[1][4]=false;controlCmndSent_goalNotReached[2][4]=false;

	readArmPathFlag[0]=false;readArmPathFlag[1]=false;readArmPathFlag[2]=false;

//	ransacFlag=false;
	pathPlanningFlag=false;
	NopathPlanningFlag=false;
	simulationFlag=false;
	controllerFlag= false;
	directPathAllocationFalg=false;
	NO_ArmState=3;
	NO_ctrlCmdType=5;

	robotCommandType=Ecmnd::e_grasp;
	pointReachNumber=0;
	armsStateFlag_NotIdle[0]=false;armsStateFlag_NotIdle[1]=false;armsStateFlag_NotIdle[2]=false;
	armstateFlag_doAction[0]=false;armstateFlag_doAction[1]=false;armstateFlag_doAction[2]=false;
//	sub_shapes			=nh.subscribe("ransac_segmentation/trackedShapes",10, &youbotCallback::CallBackShapes, this);

	sub_LeftArmQ		=nh.subscribe("Q_youbotarm"	,10,&youbotCallback::CallBackArm_Q_youbot,		this);

	sub_robotCtrlAck	=nh.subscribe("youbot_control_ack",80, &youbotCallback::SubscribeControlAck, this);
	sub_arrivingCmnd	=nh.subscribe("youbot_command",10, &youbotCallback::arrivingCommands, this);

	pub_hri_robot_ack		=nh.advertise<std_msgs::String>("youbot_ack",100);
	knowledgeBase_client	=nh.serviceClient<knowledge_msgs::knowledgeSRV>("knowledgeService");
	publishKBCommand		=nh.advertise<std_msgs::String>("robot_KB_command",80);
	publishKBCommandyoubot		=nh.advertise<std_msgs::String>("youbot_KB_command",80);
    publishKBkinect		=nh.advertise<std_msgs::String>("kinect_KB_command",80);
    publishControlCommand= nh.advertise<youbotcmd_msgs::ControlYoubot>("youbot_control_command",80);
	simYoubot_client = nh.serviceClient<simYoubot_msgs::simulateYoubotSRV>("robotSimulator_service");
	sub_arrivingSimulationCommand=nh.subscribe("simulation_command_youbot",10, &youbotCallback::arrivingSimulationCommand, this);
	pub_simulationResponse=nh.advertise<robot_interface_msgs::SimulationResponseMsg>("simulation_response_youbot",80);

	waiting_time=50.0;
	SetAgentsList();
}
youbotCallback::~youbotCallback(){
	for(int i=0;i<NoSphere+NoCylinder;i++){
		delete [] objectFeatureBox[i];
		delete [] objectFeatureBall[i];
	}
	delete [] objectFeatureBox;
}


void youbotCallback::CallBackArm_Q_youbot(const std_msgs::Float64MultiArray& msg){
	for (int i=0;i<5;i++)
		init_q_[0][i]=msg.data[i];
}





void youbotCallback::SubscribeControlAck(const youbotcmd_msgs::ControlYoubotGoalReachAck& msg) {

	ROS_INFO("I heard Control Ack: arm:%d, cmndType: %d",msg.armState,msg.ctrlCmndTypeAck);

	int arm_index=msg.armState;
	int ctrlcmndType=msg.ctrlCmndTypeAck;
	if(arm_index<NO_ArmState && ctrlcmndType<NO_ctrlCmdType)
	{
		control_goal_reach_ack[arm_index][ctrlcmndType]=true;
		agents_list[arm_index].isBusy=false;
		agents_list[arm_index].isActionSuccessfullyDone=true;

		if(agents_list[arm_index].emergencyFlag==false)
			PublishRobotAck(agents_list[arm_index]);
		else
			agents_list[arm_index].emergencyFlag=false;

	}
	else
		cout<<"youbotCallback::ControlAckCallBack==> Error in size"<<endl;


}

void youbotCallback::arrivingSimulationCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("youbotCallback::arrivingSimulationCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	// arrive the simulation command here
	// base on the arriving command call different functions
	cout<<msg.ActionName<<" ";
	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		cout<<msg.ResponsibleAgents[i]<<" ";
	for(int i=0;i<msg.ColleagueAgents.size();i++)
		cout<<msg.ColleagueAgents[i]<<" ";
	for(int i=0;i<msg.ActionParametersName.size();i++)
		cout<<msg.ActionParametersName[i]<<" ";
	cout<<endl;
	cout<<"Arm joint values:"<<endl;
	for(int i=0;i<msg.ArmsJoint.size();i++)
	{
		for(int j=0;j<5;j++)
			cout<<msg.ArmsJoint[i].values[j]<<" ";
		cout<<endl;
	}

	string tempActionName=msg.ActionName;
	if(tempActionName=="Grasp"|| tempActionName=="UnGrasp")
		SimulateGraspingCommand(msg);
	else if(tempActionName=="Stop")
		SimulateStoppingCommand(msg);
	else if(tempActionName=="HoldOn")
		SimulateHoldingCommand(msg);
	else if(tempActionName=="Approach")
		SimulateApproachingCommand(msg);
	else if(tempActionName=="UpdateJointValues")
		SimulateUpdateJointValues(msg);
	else if(tempActionName=="Rest")
		SimulateRestingcommand(msg);
	
	
	else
	{
		cout<<"The arriving msg name is wrong: "<<tempActionName <<endl;
		exit(1);
	}
}

void youbotCallback::SimulateUpdateJointValues(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("youbotCallback::SimulateUpdateJointValues"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=0.0;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	robot_interface_msgs::Joints tempJoint;
	for (int i=0;i<msg.ResponsibleAgents.size();i++)
	{
		if(msg.ResponsibleAgents[i]=="LeftArm")
		{
			for(int i=0;i<5;i++)
				tempJoint.values.push_back(init_q_[0][i]);
			tempResponseMsg.ArmsJoint.push_back(tempJoint);
		}
	
		else
		{
			cout<<"Error in arriving Msg, agent name: "<<msg.ResponsibleAgents[i]<<endl;
		}
	}

	pub_simulationResponse.publish(tempResponseMsg);
}

void youbotCallback::SimulateGraspingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("youbotCallback::SimulateGraspingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=1;
	tempResponseMsg.time=0.8;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	robot_interface_msgs::Joints tempJoint;
	for(int i=0;i<5;i++)
	tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);


	pub_simulationResponse.publish(tempResponseMsg);

};

void youbotCallback::SimulateHoldingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("youbotCallback::SimulateHoldingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=0.01;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	robot_interface_msgs::Joints tempJoint;
	for(int i=0;i<5;i++)
	tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);

	

	pub_simulationResponse.publish(tempResponseMsg);

};
void youbotCallback::SimulateStoppingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("youbotCallback::SimulateStoppingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=0.01;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	robot_interface_msgs::Joints tempJoint;
	for(int i=0;i<5;i++)
	tempJoint.values.push_back(msg.ArmsJoint[0].values[i]);
	tempResponseMsg.ArmsJoint.push_back(tempJoint);

	

	pub_simulationResponse.publish(tempResponseMsg);

};

void youbotCallback::SimulateRestingcommand(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("youbotCallback::SimulateRestingcommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

	tempResponseMsg.success=true;
	tempResponseMsg.time=5.0;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	vector<string> ResponsibleAgents;

	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);

	//! call the knowledge base
	vector<float> goalPose; int goalSize;
	knowledge_msgs::knowledgeSRV knowledge_msg;

	if(ResponsibleAgents.size()==1)
	{
		
		if(ResponsibleAgents[0]=="LeftArm")
			knowledge_msg.request.reqType="Point_RestingLeft";
		else
		{
			cout<<"Error in name of coming responsible agent "<<endl;
			exit(1);
		}
	}
	else
	{
		cout<<"Error in Resting Command Responsible Agent size"<<endl;
		exit(1);
	}


	knowledge_msg.request.Name="";
	knowledge_msg.request.requestInfo="Pose";

	if(knowledgeBase_client.call(knowledge_msg)){
		goalSize=knowledge_msg.response.pose.size();
		if(goalSize==7)
		{}
		else
		{
			cout<<"Error in number of knowledge base size"<<endl;
			exit(1);
		}
		for (int i=0;i<goalSize;i++){
			goalPose.push_back(knowledge_msg.response.pose[i]);
		}
	}


	robot_interface_msgs::Joints tempJoint;
	
	if(ResponsibleAgents[0]=="LeftArm")
	{
		for(int i=0;i<5;i++)
			tempJoint.values.push_back(goalPose[i]);
		tempResponseMsg.ArmsJoint.push_back(tempJoint);

		tempJoint.values.clear();

	}

	pub_simulationResponse.publish(tempResponseMsg);

};



void youbotCallback::SimulateApproachingCommand(const robot_interface_msgs::SimulationRequestMsg& msg){
//	cout<<BOLD(FBLU("youbotCallback::SimulateApproachingCommand"))<<endl;
	if(msg.ResponsibleAgents.size()==1)
		SimulateApproachingCommandSingleArm(msg);
	else
		cout<<"Error in agent number"<<endl;
};

void youbotCallback::SimulateApproachingCommandSingleArm(const robot_interface_msgs::SimulationRequestMsg& msg){
	cout<<BOLD(FBLU("youbotCallback::SimulateApproachingCommandSingleArm"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	// check the input msg and call the knowledge base for information we need for simulation
	//call the simulation service, wait for response and publish the result to the controller
	vector<string> msgAction;
	int agentNumber;

	if(msg.ResponsibleAgents.size()==1)
	{
		if(msg.ResponsibleAgents[0]=="LeftArm")
			agentNumber=0;
		else if(msg.ResponsibleAgents[0]=="RightArm")
			agentNumber=1;
		else
			cout<<"Error in agent name: "<<msg.ResponsibleAgents[0]<<endl;
	}
	else
	{	cout<<"The agent sizes is not defined in this function!"<<endl;
		return;
	}
	if(msg.ActionParametersName.size()>1 || msg.ActionParameterInfo.size()>1)
	{
		cout<<"Error in single approaching command, the parameter of the actions is more than one: "<<msg.ActionParametersName.size()<<msg.ActionParameterInfo.size()<<endl;
	}

	string actionParameters= msg.ActionParametersName[0]; //Approach [-]: point5, Object1-graspingPose1, Object4-XPose3
	vector<string> actionParametersVec;
	boost::split(actionParametersVec,actionParameters, boost::is_any_of("-"));// point5 [ Approach_point5] Cylinder1-graspingPose1 [ Approach_Cylinder1-graspingPose1]




	vector<float> initialJointPose, finalJointPose;
	finalJointPose.resize(num_joint,0.0);
	double actionTime=0;
	bool simulationResult;
	for(int i=0;i<msg.ArmsJoint[agentNumber].values.size();i++)
		initialJointPose.push_back(msg.ArmsJoint[agentNumber].values[i]);

	//SimulateServiceApproachSingleArm(agentNumber, initialJointPose , goalPose, simulationResult, actionTime, finalJointPose);

    finalJointPose = initialJointPose;
	robot_interface_msgs::SimulationResponseMsg tempResponseMsg;

//	tempResponseMsg.success=simulationResult;
//	tempResponseMsg.time=actionTime;//sec
//	tempResponseMsg.ActionName=msg.ActionName;

	tempResponseMsg.success=true;
	tempResponseMsg.time=2;//sec
	tempResponseMsg.ActionName=msg.ActionName;

	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		tempResponseMsg.ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);
	for(int i=0;i<msg.ActionParameterInfo.size();i++)
		tempResponseMsg.ActionParameterInfo.push_back(msg.ActionParameterInfo[i]);
	for(int i=0;i<msg.ActionParametersName.size();i++)
		tempResponseMsg.ActionParametersName.push_back(msg.ActionParametersName[i]);
	for(int i=0;i<msg.ColleagueAgents.size();i++)
		tempResponseMsg.ColleagueAgents.push_back(msg.ColleagueAgents[i]);




	robot_interface_msgs::Joints leftArmJoint,rightArmJoint;
	for(int i=0;i<5;i++)
	{
		if(agentNumber==0)
		{
			//leftArmJoint.values.push_back(0.0);
			leftArmJoint.values.push_back(finalJointPose[i]);
		}
		
	}
	cout<<"final q: "<<endl;
	for(int i=0;i<5;i++)
		cout<<leftArmJoint.values[i]<<" ";
	cout<<endl;



	tempResponseMsg.ArmsJoint.push_back(leftArmJoint);


	pub_simulationResponse.publish(tempResponseMsg);
//	exit(1);
};




void youbotCallback::arrivingCommands(const std_msgs::String::ConstPtr& input1){
	// MSG: "[action] [agents who should perform] [collaborators]"
	// Example:  Approach-Point-11 LeftArm+RightArm Human
	cout<<BOLD(FBLU("youbotCallback::arrivingCommands"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	string input=input1-> data.c_str();
	ROS_INFO("Arrived robot command: %s",input.c_str());
	int agentNumber;
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint ;

//	cout<<"101"<<endl;
	boost::split(msg, input, boost::is_any_of(" "));
//	cout<<"101-1"<<endl;
	boost::split(msgAction, msg[0], boost::is_any_of("_"));
//	cout<<"101-2"<<endl;

	boost::split(msgAgents, msg[1], boost::is_any_of("+"));
//	cout<<"101-3"<<endl;

	if(msg.size()==3)
	boost::split(msgColleagues, msg[2], boost::is_any_of("+"));
	else if(msg.size()>3)
		cout<<"Error in arriving msg size: "<<msg.size()<<input<<endl;

	//! first find which agents should perform the action and assign it.
//	cout<<"102"<<endl;
	if(msgAgents.size()==1)
	{
		if(msgAgents[0]=="LeftArm")
		{
			agentNumber=0;
		}
		
		else
		{
			cout<<"The agents definition is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
		}
	}

	
	else
	{
		cout<<"The agents size is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
	}
//	cout<<"103: "<<agentNumber<<agents_list.size()<<endl;

	//***********************************************************
	// fill the related agent with the action
	if(agents_list[agentNumber].isBusy==false || msg[0]=="Stop")
	{
//		cout<<"103-1"<<endl;

		agents_list[agentNumber].isBusy=true;
//		cout<<"103-2"<<endl;
		agents_list[agentNumber].lastAssignedAction=msg[0];
//		cout<<"103-3"<<endl;
		agents_list[agentNumber].microSec_StartingTime=duration_cast< microseconds >(system_clock::now().time_since_epoch());
//		cout<<"103-4"<<endl;
		agents_list[agentNumber].collaborators=msgColleagues;

	}
	else
	{
		cout<<"The agent you assigned is busy now and it can not perform a new action "<<endl;
		// publish false to the planner for the arrived action
	}
//	cout<<"104"<<endl;

	//******************************************************
	// find the correct function for each action

	if(msgAction[0]=="Approach")
	{
		SendApproachingCommand(agents_list[agentNumber]);
	}
	
	else if(msgAction[0]=="Rest")
	{
		SendRestingCommand(agents_list[agentNumber]);
	}
	
	else if(msgAction[0]=="Grasp" || msgAction[0]=="UnGrasp")
	{
		SendGraspingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="Stop")
	{
		SendStoppingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="HoldOn")
	{
		SendHoldingCommand(agents_list[agentNumber]);
	}
	
	
	else
	{
		cout<<"Error in arriving msg action:"<<msgAction[0]<<endl;
	}



}


void youbotCallback::PublishRobotAck(agents_tasks& agent){
	cout<<BOLD(FBLU("youbotCallback::PublishRobotAck"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	std_msgs::String ackMsg;
	cout<<"Last assigned action: "<<agent.lastAssignedAction<<endl;
	ackMsg.data=agent.lastAssignedAction+" "; // Appraoch_Point2, Grasp, Transport_Cylinder2-GraspingPose1_Point7

	for(int i=0;i<agent.agents.size();i++){
		ackMsg.data+=agent.agents[i];
		if(i<agent.agents.size()-1)
			ackMsg.data=ackMsg.data+"+";
	}
	if(agent.isActionSuccessfullyDone==true)
		ackMsg.data+=" true";
	else
		ackMsg.data+=" false";

	if (agent.emergencyFlag==false)
		pub_hri_robot_ack.publish(ackMsg);
	else
	{
		cout<<"The agent emergency flag is true, therefore we do not give ack to the planner"<<endl;
		agent.Print();
	}
}

void youbotCallback::SetAgentsList(){
	agents_tasks agent1;
	agent1.agents.push_back("LeftArm");
	agent1.agentsNumber=0;
	agent1.collaborators.clear();
	agents_list.push_back(agent1);



};



void youbotCallback::SendGraspingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("youbotCallback::sendGraspingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	agent.Print();

	control_msg.Activation=2;
	control_msg.youbotArm.youbotIndex=agent.agentsNumber;

	if(agent.lastAssignedAction=="Grasp")
		control_msg.youbotArm.value=0;
	else if(agent.lastAssignedAction=="UnGrasp")
		control_msg.youbotArm.value=1;
	else
		cout<<"Error in assigned action: "<<agent.lastAssignedAction<<endl;
	publishControlCommand.publish(control_msg);

};
void youbotCallback::SendHoldingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("youbotCallback::sendHoldingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	control_msg.Activation=4;
	control_msg.youbotArm.youbotIndex=agent.agentsNumber;
	control_msg.youbotArm.value=1; // hold on=1, hold off=0
	publishControlCommand.publish(control_msg);

};
void youbotCallback::SendStoppingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("youbotCallback::sendStoppingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	agent.Print();

	control_msg.Activation=3;
	control_msg.youbotArm.youbotIndex=agent.agentsNumber;
	publishControlCommand.publish(control_msg);

};

void youbotCallback::SendApproachingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("youbotCallback::SendApproachingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	if(agent.agentsNumber==0 || agent.agentsNumber==1)
		SendApproachingCommandSingleArm(agent);
	else if (agent.agentsNumber==2)
	{
		//SendTransportingCommandJointArms(agent);
	}
	else
		cout<<"Error in agent number"<<endl;
};


void youbotCallback::SendRestingCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("youbotCallback::SendRestingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	/* 1- Parse the assigned action
	   2- get from the knowledge base the necessary info
	   3- base on the flag: find the path for the robot end effector
	   4- base on the flag: simulate the robot behavior based on the path
	   5- base on the flag: give command to the controller
	 */

	agent.Print();

	//! parse the input command
	vector<string> msgAction, msgParameters;
	vector<float> goalPose;
	int goalSize;
	if(agent.agentsNumber==0)
		msgParameters.push_back("Point_RestingLeft");
	
	else
		cout<<"Error in agent number"<<endl;

	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg;

	knowledge_msg.request.reqType=msgParameters[0];
	if(msgParameters.size()>1)
		knowledge_msg.request.Name=msgParameters[1];
	else
		knowledge_msg.request.Name="";
	knowledge_msg.request.requestInfo="Pose";

	if(knowledgeBase_client.call(knowledge_msg)){

		goalSize=knowledge_msg.response.pose.size();

		if(goalSize==6)
			control_msg.youbotArm.youbotCmdType="cartPos";
		else if(goalSize==5)
			control_msg.youbotArm.youbotCmdType="jointPos";
		else
		{
			cout<<"Error in number of knowledge base size"<<endl;
			exit(1);
		}

		for (int i=0;i<goalSize;i++){
			goalPose.push_back(knowledge_msg.response.pose[i]);
		}

		simulationFlag=true;
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	cout<<"goalPose: ";
	for (int i=0;i<goalSize;i++)
	{
			cout<<goalPose[i]<<" ";
	}
	cout<<endl;

		control_msg.Activation=0;
		control_msg.youbotArm.youbotIndex=agent.agentsNumber;
		control_msg.youbotArm.youbotCmdType="jointPos";
		for(int i=0;i<goalSize;i++)
			control_msg.youbotArm.cartGoal.youbotJointPose[i]=goalPose[i];
		publishControlCommand.publish(control_msg);
};





void youbotCallback::SendApproachingCommandSingleArm(agents_tasks& agent){
	cout<<BOLD(FBLU("youbotCallback::SendApproachingCommandSingleArm"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	/* 1- Parse the assigned action
	   2- get from the knowledge base the necessary info
	   3- base on the flag: find the path for the robot end effector
	   4- base on the flag: simulate the robot behavior based on the path
	   5- base on the flag: give command to the controller

	 */
	agent.Print();
	bool simulationResult;

	//! parse the input command
	vector<string> msgAction, msgParameters;
	boost::split(msgAction,agent.lastAssignedAction, boost::is_any_of("_"));// Approach_Point5, Approach_Cylinder1-graspingPose2
	boost::split(msgParameters, msgAction[1], boost::is_any_of("-"));// Point5, Cylinder1-graspingPose2
	vector<float> goalPose;
	int goalSize;

	//! call the knowledge base
	knowledge_msgs::knowledgeSRV knowledge_msg;


	std_msgs::String kbcbmsg;
	kbcbmsg.data=msgAction[1];
	publishKBCommandyoubot.publish(kbcbmsg);
	

		

		if(msgAction[1]=="gotonextobject"){
			control_msg.Activation=5;
			control_msg.youbotArm.youbotIndex=agent.agentsNumber;
			control_msg.youbotArm.youbotCmdType="";
			kbcbmsg.data = "removeobject";
			publishControlCommand.publish(control_msg);
			publishKBCommandyoubot.publish(kbcbmsg);
			
		}
		else if(msgAction[1]=="gotohuman"){
			control_msg.Activation=5;
			control_msg.youbotArm.youbotIndex=agent.agentsNumber;
			control_msg.youbotArm.youbotCmdType="";
			publishControlCommand.publish(control_msg);
		}
		else {
            knowledge_msg.request.reqType=msgAction[1];
			knowledge_msg.request.Name="";
	        knowledge_msg.request.requestInfo="Pose";

	if(knowledgeBase_client.call(knowledge_msg)){
			
            	
         goalSize=knowledge_msg.response.pose.size();
         for (int i=0;i<goalSize;i++){
			goalPose.push_back(knowledge_msg.response.pose[i]);
		}
		

        cout<<"goalPose: ";
	          for (int i=0;i<goalSize;i++){
			      cout<<goalPose[i]<<" ";
		                            }
	          cout<<endl;


		control_msg.Activation=0;
		control_msg.youbotArm.youbotIndex=agent.agentsNumber;
		control_msg.youbotArm.youbotCmdType="jointPos";
		for(int i=0;i<goalSize;i++)
		control_msg.youbotArm.cartGoal.youbotJointPose[i]=goalPose[i];
		publishControlCommand.publish(control_msg);

		}}
	

		

		simulationFlag=true;

};




void youbotCallback::FailureCheck(void){


	microseconds microSec_CurrentTime=duration_cast< microseconds >(system_clock::now().time_since_epoch());

	for(int i=0;i<agents_list.size();i++)
	{
		if(agents_list[i].isBusy==true)
		{
			if( (microSec_CurrentTime.count()-agents_list[i].microSec_StartingTime.count())/1000000.0 > waiting_time ) //value of time in micro seconds
			{
				cout<<"youbotCallback::FailureCheck"<<endl;
				cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

				cout<<"waiting time from last command: "<<(microSec_CurrentTime.count()-agents_list[i].microSec_StartingTime.count())/1000000.0<<endl;
				agents_list[i].isBusy=false;
				agents_list[i].isActionSuccessfullyDone=false;
				PublishRobotAck(agents_list[i]);
				// also stop the robot for the current action
				StopRobotEmergency(agents_list[i]);

			}
		}

	}

};

void youbotCallback::StopRobotEmergency(agents_tasks& agent){
	cout<<BOLD(FBLU("youbotCallback::StopRobotEmergency"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	vector<string> msgColleagues;

	agent.isBusy=true;
	agent.lastAssignedAction="Stop";
	agent.microSec_StartingTime=duration_cast< microseconds >(system_clock::now().time_since_epoch());
	agent.collaborators=msgColleagues;
	agent.emergencyFlag=true;
	SendStoppingCommand(agent);

};


