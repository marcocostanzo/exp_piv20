#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sun_plan_msgs/RLabplannerAction.h>
#include <iiwa_interp/RLabinterpolationAction.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_msgs/Float64MultiArray.h"
#include <iiwa_msgs/JointPosition.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "slipping_control_common/Slipping_Control_Client.h"

#define GRIPPER_ACTIVE true
#define SEQUENCE_GRASP 2
#define SEQUENCE_RELEASE 3
#define SEQUENCE_GRIPPER_PIV 1
#define SEQUENCE_SA 0

using namespace std;

char askForChar( const char* str){
	/* USAGE
	char ans = askForChar( "Continue? [y = YES / n = nextIter / e = exit ]: " );
		switch( ans ){
			case 'n' :
			case 'N' :
				continue;
			case 'y' :
			case 'Y' :
			case 's' :
			case 'S' :
				break;
			default:
				exit(1);			
		}
		ans = 0;
	*/

	char ans;
	cout << str;
	cin >> ans;
	cout << endl;

	return ans;
}


// Params to be read from the launch file
int NUM_ITER, num_joints;
double 	objDimX, objDimY, objDimZ,
	objPosX, objPosY, objPosZ,
	hTable, hShelf,
	pickAngleX, pickAngleY, pickAngleZ,
   prePlaceAngleX, prePlaceAngleY, prePlaceAngleZ,
   placeAngleX, placeAngleY, placeAngleZ,
   prePickPosZdelta, pickPosXdelta, pickPosYdelta, pickPosZdelta,
   prePlaceXdelta, prePlaceYdelta, prePlaceZdelta,
   placePosX, placePosY, placePosZdelta, hShelfMultipler,
   safeRetraitXdelta, safeRetraitYdelta, safeRetraitZdelta,
	dim1_position_constraint;

std::string 	group_name,
		path_urdf_model,
		path_urdf_augmented,
		link_ee_name,
		link_dummy_name;
bool activate_pivoting;
bool start_conf_acquired = false;
ros::Subscriber iiwa_current_state;

geometry_msgs::Pose target_pose;
tf2::Quaternion orientation;
moveit_msgs::Constraints path_constraints;
moveit_msgs::Constraints path_constraints_final;
moveit_msgs::OrientationConstraint ocm;
bool attached_object = false;
bool detach_object = false;
bool finished_before_timeout = false;
std::vector<double> current_joint_values;
std::vector<double> start_joint_values;

sun_plan_msgs::RLabplannerGoal plannerGoal;
iiwa_interp::RLabinterpolationGoal interpGoal;

void getCurretRobotConfig(std::string group_name_)
{
	moveit::planning_interface::MoveGroupInterface group(group_name_);
	robot_state::RobotState start_state(*group.getCurrentState());
	const robot_model::JointModelGroup* joint_model_group = start_state.getJointModelGroup(group_name_);
	start_state.copyJointGroupPositions(joint_model_group, current_joint_values);
	ROS_INFO("CURRENT JOINT CONFIGURATION ---> current_joint_values.size() = %d", current_joint_values.size());
	ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f", 		current_joint_values[0], 
						   		current_joint_values[1], 
								current_joint_values[2], 
								current_joint_values[3], 
								current_joint_values[4],
								current_joint_values[5], 
								current_joint_values[6], 
								current_joint_values[7]);
}

void setInitialPosition()
  {

    ros::NodeHandle nh;
    sleep(2);
    ros::Publisher fake_state = nh.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);	
    sensor_msgs::JointState joint_state;
        joint_state.name.push_back("iiwa_joint_1");
	joint_state.name.push_back("iiwa_joint_2");
	joint_state.name.push_back("iiwa_joint_3");
	joint_state.name.push_back("iiwa_joint_4");
	joint_state.name.push_back("iiwa_joint_5");
	joint_state.name.push_back("iiwa_joint_6");
	joint_state.name.push_back("iiwa_joint_7");
	joint_state.position.push_back(start_joint_values[0]);
	joint_state.position.push_back(start_joint_values[1]);
	joint_state.position.push_back(start_joint_values[2]);
	joint_state.position.push_back(start_joint_values[3]);
	joint_state.position.push_back(start_joint_values[4]);
	joint_state.position.push_back(start_joint_values[5]);
	joint_state.position.push_back(start_joint_values[6]); 
    for(int i = 0; i < 3; i++)
	{
		fake_state.publish(joint_state);
		sleep(1);
	}
char ans = askForChar( "Initial confing set - Continue? [y = YES / n = no / e = exit ]: " );
		switch( ans ){
			case 'n' :
			case 'N' :{
				exit(-1);
			        break;}
			case 'y' :
			case 'Y' :
			case 's' :
			case 'S' :
				break;
			default:
				exit(1);			
		}
		ans = 0;
 }

void fillPlannerActionMsg(	const std::string& group_name_,
			int num_joints_,
			const geometry_msgs::Pose& target_pose_,
			const moveit_msgs::Constraints& path_constraints_,
			bool activate_pivoting_,
			const std::string& path_urdf_model_,
			const std::string& path_urdf_augmented_,
			const std::string& link_ee_name_,
			const std::string& link_dummy_name_
			)
{
	plannerGoal.group_name 			= group_name_;
	plannerGoal.num_joints 			= num_joints_;
	plannerGoal.target_pose 			= target_pose_;
	plannerGoal.path_constraints 			= path_constraints_;
	plannerGoal.activate_pivoting 			= activate_pivoting_;
	plannerGoal.path_urdf_model 			= path_urdf_model_;
	plannerGoal.path_urdf_augmented 		= path_urdf_augmented_;
	plannerGoal.link_ee_name 			= link_ee_name_;
	plannerGoal.link_dummy_name	 		= link_dummy_name_;
	getCurretRobotConfig(group_name_);
	plannerGoal.start_config.clear();
	for (int p = 0; p < num_joints_; p++)
		plannerGoal.start_config.push_back(current_joint_values[p]);

}

void fillInterpolationActionMsg( const trajectory_msgs::JointTrajectory& planned_trajectory )
{

  interpGoal.planned_trajectory = planned_trajectory;

}

void fill_vectors( const sun_plan_msgs::RLabplannerResultConstPtr& result,  std::vector< trajectory_msgs::JointTrajectory >& all_traj, std::vector<int>& sequence_vec )
{
  for( auto & traj : result->planned_trajectories )
  {
    all_traj.push_back( traj );
  }
  for( auto & seq : result->sequence )
  {
    sequence_vec.push_back( seq );
  }
}

void modify_last_sequence( const sun_plan_msgs::RLabplannerResultConstPtr& result, int sequence_type , std::vector<int>& sequence_vec )
{
  sequence_vec[ sequence_vec.size() - result->planned_trajectories.size() ] = sequence_type;
}



void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "world";
	collision_object.id = "object";
	collision_object.primitives.resize(1);
	collision_object.primitives[0].type = collision_object.primitives[0].BOX;
	collision_object.primitives[0].dimensions.resize(3);
	collision_object.primitives[0].dimensions[0] = objDimX;
	collision_object.primitives[0].dimensions[1] = objDimY;
	collision_object.primitives[0].dimensions[2] = objDimZ;
	collision_object.primitive_poses.resize(1);
	collision_object.primitive_poses[0].position.x = objPosX;
	collision_object.primitive_poses[0].position.y = objPosY;
	collision_object.primitive_poses[0].position.z = hTable + hShelf + objPosZ;
	collision_object.operation = collision_object.ADD;
	ROS_INFO("Creating BOX object ('object') in the world scene");
	planning_scene_interface.applyCollisionObject(collision_object);
}

void readJointPos(const iiwa_msgs::JointPosition jointStateMsg)
{
	start_joint_values[0] = jointStateMsg.position.a1;
	start_joint_values[1] = jointStateMsg.position.a2;
	start_joint_values[2] = jointStateMsg.position.a3;
	start_joint_values[3] = jointStateMsg.position.a4;
	start_joint_values[4] = jointStateMsg.position.a5;
	start_joint_values[5] = jointStateMsg.position.a6;
	start_joint_values[6] = jointStateMsg.position.a7;
        //ROS_INFO("ACQUISITA!!!!!!!!!!!!!!!");
	start_conf_acquired = true;	
	//iiwa_current_state.shutdown();
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "rlabplanner_client_denkmit");
  ros::NodeHandle nh;
  ros::NodeHandle nodehandle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Read the params values from the launch file
  nodehandle.getParam("objDimX", objDimX);
  nodehandle.getParam("objDimY", objDimY);
  nodehandle.getParam("objDimZ", objDimZ);
  nodehandle.getParam("objPosX", objPosX);
  nodehandle.getParam("objPosY", objPosY);
  nodehandle.getParam("objPosZ", objPosZ);

  nodehandle.getParam("prePickPosZdelta", prePickPosZdelta);

  nodehandle.getParam("pickPosXdelta", pickPosXdelta);
  nodehandle.getParam("pickPosYdelta", pickPosYdelta);
  nodehandle.getParam("pickPosZdelta", pickPosZdelta);
  nodehandle.getParam("hTable", hTable);
  nodehandle.getParam("hShelf", hShelf);
  nodehandle.getParam("pickAngleX", pickAngleX);
  nodehandle.getParam("pickAngleY", pickAngleY);
  nodehandle.getParam("pickAngleZ", pickAngleZ);

  nodehandle.getParam("prePlaceAngleX", prePlaceAngleX);
  nodehandle.getParam("prePlaceAngleY", prePlaceAngleY);
  nodehandle.getParam("prePlaceAngleZ", prePlaceAngleZ);
  nodehandle.getParam("placeAngleX", placeAngleX);
  nodehandle.getParam("placeAngleY", placeAngleY);
  nodehandle.getParam("placeAngleZ", placeAngleZ);
  nodehandle.getParam("prePlaceXdelta", prePlaceXdelta);
  nodehandle.getParam("prePlaceYdelta", prePlaceYdelta);
  nodehandle.getParam("prePlaceZdelta", prePlaceZdelta);
  nodehandle.getParam("placePosX", placePosX);
  nodehandle.getParam("placePosY", placePosY);
  nodehandle.getParam("placePosZdelta", placePosZdelta);
  nodehandle.getParam("hShelfMultipler", hShelfMultipler);

  nodehandle.getParam("safeRetraitXdelta", safeRetraitXdelta);
  nodehandle.getParam("safeRetraitYdelta", safeRetraitYdelta);
  nodehandle.getParam("safeRetraitZdelta", safeRetraitZdelta);

  nodehandle.getParam("dim1_position_constraint", dim1_position_constraint);
  nodehandle.getParam("group_name", group_name);
  nodehandle.getParam("num_joints", num_joints);
  nodehandle.getParam("activate_pivoting", activate_pivoting);
  nodehandle.getParam("path_urdf_model", path_urdf_model);
  nodehandle.getParam("path_urdf_augmented", path_urdf_augmented);
  nodehandle.getParam("link_ee_name", link_ee_name);
  nodehandle.getParam("link_dummy_name", link_dummy_name);

  // True causes the client to spin its own thread
  actionlib::SimpleActionClient<sun_plan_msgs::RLabplannerAction> ac_planner("rlabplanner", true);
  actionlib::SimpleActionClient<iiwa_interp::RLabinterpolationAction> ac_interp("rlabinterpolation", true);
  sun_plan_msgs::RLabplannerResult result_;
  sun_plan_msgs::RLabplannerFeedback feedback_;
  ros::Rate* loop_rate = new ros::Rate(1000); 

  Slipping_Control_Client slipping_control(nodehandle, GRIPPER_ACTIVE);

  slipping_control.home();

  ROS_INFO("Waiting for action PLANNER ACTION SERVER to start.");
  ac_planner.waitForServer();
  ROS_INFO("PLANNER ACTION SERVER started, sending plannerGoal.");

  ROS_INFO("Waiting for action INTERPOLATION ACTION SERVER to start.");
  ac_interp.waitForServer();
  ROS_INFO("INTERPOLATION ACTION SERVER started, sending plannerGoal.");

  // Starting the pipeline
  iiwa_current_state = nh.subscribe("/iiwa/state/JointPosition", 1, readJointPos);

  ROS_INFO("Read current robot start configuration..");
  start_joint_values.resize(7);
  while(start_conf_acquired == false)
  {
     ros::spinOnce();   
     loop_rate->sleep();  
  }
  iiwa_current_state.shutdown();

  setInitialPosition();

  moveit::planning_interface::MoveGroupInterface group(group_name);
  current_joint_values.resize(num_joints);
  for(int i = 0; i < num_joints; i++)
  	current_joint_values[i] = 0.0;

  //Add the object to be picked into the world scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  addCollisionObject(planning_scene_interface);
  ros::WallDuration(1.0).sleep();

  //Initialize trajs
  std::vector< trajectory_msgs::JointTrajectory > all_traj;
  std::vector<int> sequence_vec;

  // PRE-PICK
  orientation.setRPY(pickAngleX, pickAngleY, pickAngleZ);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = objPosX + pickPosXdelta;
  target_pose.position.y = objPosY + pickPosYdelta;
  target_pose.position.z = hTable + hShelf + pickPosZdelta + objPosZ + prePickPosZdelta;
  fillPlannerActionMsg(group_name, num_joints, target_pose, path_constraints, activate_pivoting, path_urdf_model, path_urdf_augmented, link_ee_name, link_dummy_name);
  ac_planner.sendGoal(plannerGoal);
  finished_before_timeout = ac_planner.waitForResult(ros::Duration(180.0));  //wait for the action to return
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState plannerState = ac_planner.getState();
    ROS_INFO("Action finished: %s", plannerState.toString().c_str());
  }
  else{
    ROS_INFO("Action did not finish before the time out.");
    exit(-1);
  }

  fill_vectors( ac_planner.getResult(),  all_traj, sequence_vec );



  // PICK
  orientation.setRPY(pickAngleX, pickAngleY, pickAngleZ);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = objPosX + pickPosXdelta;
  target_pose.position.y = objPosY + pickPosYdelta;
  target_pose.position.z = hTable + hShelf + pickPosZdelta + objPosZ;
  fillPlannerActionMsg(group_name, num_joints, target_pose, path_constraints, activate_pivoting, path_urdf_model, path_urdf_augmented, link_ee_name, link_dummy_name);
  ac_planner.sendGoal(plannerGoal);
  finished_before_timeout = ac_planner.waitForResult(ros::Duration(180.0));  //wait for the action to return
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState plannerState = ac_planner.getState();
    ROS_INFO("Action finished: %s", plannerState.toString().c_str());
    }
  else{
    ROS_INFO("Action did not finish before the time out.");
    exit(-1);
  }

  fill_vectors( ac_planner.getResult(),  all_traj, sequence_vec );

  // ATTACHED OBJECT 
  attached_object = group.attachObject("object", group.getEndEffectorLink().c_str());
  ROS_INFO("Attached_object (1:true - 0:false) = %d", (int)attached_object);
  ros::WallDuration(1.0).sleep();


  // PICK-RETREAT
  orientation.setRPY(pickAngleX, pickAngleY, pickAngleZ);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = objPosX + pickPosXdelta;
  target_pose.position.y = objPosY + pickPosYdelta;
  target_pose.position.z = hTable + hShelf + pickPosZdelta + objPosZ + prePickPosZdelta;
  ROS_INFO("Applico il constraint in orientamento sul dummy");
  /*ocm.link_name = group.getEndEffectorLink().c_str();
  ocm.header.frame_id = group.getEndEffectorLink().c_str();
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.0175;
  ocm.absolute_y_axis_tolerance = 0.0175;
  ocm.absolute_z_axis_tolerance = 0.0175;
  ocm.weight = 1.0;
  path_constraints.orientation_constraints.push_back(ocm);*/
  fillPlannerActionMsg(group_name, num_joints, target_pose, path_constraints, activate_pivoting, path_urdf_model, path_urdf_augmented, link_ee_name, link_dummy_name);
  ac_planner.sendGoal(plannerGoal);
  finished_before_timeout = ac_planner.waitForResult(ros::Duration(180.0));  //wait for the action to return
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState plannerState = ac_planner.getState();
    ROS_INFO("Action finished: %s", plannerState.toString().c_str());
    }
  else{
    ROS_INFO("Action did not finish before the time out.");
    exit(-1);
  }
  fill_vectors( ac_planner.getResult(),  all_traj, sequence_vec );
  modify_last_sequence( ac_planner.getResult(), SEQUENCE_GRASP ,sequence_vec );

//goto execute;

  // PRE-PLACE
  orientation.setRPY(prePlaceAngleX, prePlaceAngleY, prePlaceAngleZ);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = placePosX + prePlaceXdelta;
  target_pose.position.y = placePosY + prePlaceYdelta;
  target_pose.position.z = hTable + hShelfMultipler*hShelf + objDimX/2.0 + fabs(pickPosXdelta) + prePlaceZdelta;
  ROS_INFO("Applico il constraint in orientamento sul dummy");
  ocm.link_name = group.getEndEffectorLink().c_str();
  ocm.header.frame_id = group.getEndEffectorLink().c_str();
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.0175;
  ocm.absolute_y_axis_tolerance = 0.0175;
  ocm.absolute_z_axis_tolerance = M_PI/2.0;
  ocm.weight = 1.0;
  path_constraints.orientation_constraints.push_back(ocm);
  fillPlannerActionMsg(group_name, num_joints, target_pose, path_constraints, activate_pivoting, path_urdf_model, path_urdf_augmented, link_ee_name, link_dummy_name);
  ac_planner.sendGoal(plannerGoal);
  finished_before_timeout = ac_planner.waitForResult(ros::Duration(180.0));  //wait for the action to return
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState plannerState = ac_planner.getState();
    ROS_INFO("Action finished: %s", plannerState.toString().c_str());
    }
  else{
    ROS_INFO("Action did not finish before the time out.");
    exit(-1);
  }
  fill_vectors( ac_planner.getResult(),  all_traj, sequence_vec );

  // PLACE
  orientation.setRPY(placeAngleX, placeAngleY, placeAngleZ);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = placePosX;
  target_pose.position.y = placePosY;
  target_pose.position.z = hTable + hShelfMultipler*hShelf + objDimX/2.0 + fabs(pickPosXdelta) + placePosZdelta;
  ROS_INFO("Applico il constraint in orientamento sul dummy");
  ocm.link_name = group.getEndEffectorLink().c_str();
  ocm.header.frame_id = group.getEndEffectorLink().c_str();
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.0175;
  ocm.absolute_y_axis_tolerance = 0.0175;
  ocm.absolute_z_axis_tolerance = 0.0175;
  ocm.weight = 1.0;
  path_constraints.orientation_constraints.push_back(ocm);
  fillPlannerActionMsg(group_name, num_joints, target_pose, path_constraints, activate_pivoting, path_urdf_model, path_urdf_augmented, link_ee_name, link_dummy_name);
  ac_planner.sendGoal(plannerGoal);
  finished_before_timeout = ac_planner.waitForResult(ros::Duration(1800.0));  //wait for the action to return
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState plannerState = ac_planner.getState();
    ROS_INFO("Action finished: %s", plannerState.toString().c_str());
    }
  else{
    ROS_INFO("Action did not finish before the time out.");
    exit(-1);
  }
  fill_vectors( ac_planner.getResult(),  all_traj, sequence_vec );

  // DETACH OBJECT
  ros::WallDuration(2.0).sleep();
  ROS_INFO("Open the gripper . . .");
  detach_object = group.detachObject("object");
  ROS_INFO("detach_object (1:true - 0:false) = %d", (int)detach_object);

  // RETREAT
  orientation.setRPY(prePlaceAngleX, prePlaceAngleY, prePlaceAngleZ);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = placePosX + prePlaceXdelta;
  target_pose.position.y = placePosY + prePlaceYdelta;
  target_pose.position.z = hTable + hShelfMultipler*hShelf + objDimX/2.0 + fabs(pickPosXdelta) + prePlaceZdelta;
  fillPlannerActionMsg(group_name, num_joints, target_pose, path_constraints_final, activate_pivoting, path_urdf_model, path_urdf_augmented, link_ee_name, link_dummy_name);
  ac_planner.sendGoal(plannerGoal);
  finished_before_timeout = ac_planner.waitForResult(ros::Duration(180.0));  //wait for the action to return
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState plannerState = ac_planner.getState();
    ROS_INFO("Action finished: %s", plannerState.toString().c_str());
    }
  else{
    ROS_INFO("Action did not finish before the time out.");
    exit(-1);
  }
  fill_vectors( ac_planner.getResult(),  all_traj, sequence_vec );
  modify_last_sequence( ac_planner.getResult(), SEQUENCE_RELEASE ,sequence_vec );


  // SAFE-RETREAT
  orientation.setRPY(prePlaceAngleX, prePlaceAngleY, prePlaceAngleZ);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = placePosX + prePlaceXdelta + safeRetraitXdelta;
  target_pose.position.y = placePosY + prePlaceYdelta + safeRetraitYdelta;
  target_pose.position.z = hTable + hShelfMultipler*hShelf + objDimX/2.0 + fabs(pickPosXdelta) + prePlaceZdelta + safeRetraitZdelta;
  fillPlannerActionMsg(group_name, num_joints, target_pose, path_constraints_final, activate_pivoting, path_urdf_model, path_urdf_augmented, link_ee_name, link_dummy_name);
  ac_planner.sendGoal(plannerGoal);
  finished_before_timeout = ac_planner.waitForResult(ros::Duration(180.0));  //wait for the action to return
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState plannerState = ac_planner.getState();
    ROS_INFO("Action finished: %s", plannerState.toString().c_str());
    }
  else{
    ROS_INFO("Action did not finish before the time out.");
    exit(-1);
  }
  fill_vectors( ac_planner.getResult(),  all_traj, sequence_vec );

execute:
  //ESECUZIONE
  for( int i=0; i<all_traj.size(); i++ )
  {
cout << "traj " << i << "last time = " << all_traj[i].points.back().time_from_start.toSec() << endl;
char ans = askForChar( "Continue? [y = YES / n = no / e = exit ]: " );
		switch( ans ){
			case 'n' :
			case 'N' :{
				exit(-1);
			        break;}
			case 'y' :
			case 'Y' :
			case 's' :
			case 'S' :
				break;
			default:
				exit(1);			
		}
		ans = 0;
    switch (sequence_vec[i])
    {
      case SEQUENCE_SA:{
        cout << "MOTION_SLIPPING_AVOIDANCE" << endl;
        if( slipping_control.is_grasped() )
          slipping_control.slipping_avoidance(true);
        break;
      }
      case SEQUENCE_GRIPPER_PIV:{
        cout << "SEQUENCE_GRIPPER_PIV" << endl;
        if( slipping_control.is_grasped() )
          slipping_control.gripper_pivoting();
        break;
      }
      case SEQUENCE_GRASP:{
        cout << "SEQUENCE_GRASP" << endl;
        slipping_control.grasp(2.0);
		ans = askForChar( "Continue? [y = YES / n = no / e = exit ]: " );
		switch( ans ){
			case 'n' :
			case 'N' :{
				exit(-1);
			        break;}
			case 'y' :
			case 'Y' :
			case 's' :
			case 'S' :
				break;
			default:
				exit(1);			
		}
		ans = 0;
        slipping_control.slipping_avoidance(true);
        break;
      }
      case SEQUENCE_RELEASE:{
        cout << "SEQUENCE_RELEASE" << endl;
        slipping_control.grasp(0.0);
        slipping_control.home();
        break;
      }
      default:{
        cout << "INVALID SEQUENCE" << endl;
        exit(-1);
      }
    }

    fillInterpolationActionMsg( all_traj[i] );
cout << "traj " << i << "sending..." << endl;
    ac_interp.sendGoal(interpGoal);
cout << "traj " << i << "wait" << endl;
    ac_interp.waitForResult();
cout << "traj " << i << "end" << endl;

    if( !ac_interp.getResult()->interpolation_success ){
      cout << "Error in interpolator" << endl;
      exit(-1);
    }
    
  }

  //exit
  return 0;
}
