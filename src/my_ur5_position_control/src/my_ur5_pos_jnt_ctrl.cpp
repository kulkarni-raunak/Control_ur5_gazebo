#include <ros/ros.h>
#include <ros/package.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include "control_msgs/JointControllerState.h"

const int num_of_joints = 6;
KDL::JntArray start_of_joint_pos(num_of_joints);

void Get_value_shoulder_pan_joint(const control_msgs::JointControllerState::ConstPtr& MSGs) {

	start_of_joint_pos(0) = MSGs->process_value;
}

void Get_value_shoulder_arm_joint(const control_msgs::JointControllerState::ConstPtr& MSGs) {

   	start_of_joint_pos(1) = MSGs->process_value;
}

void Get_value_elbow_joint(const control_msgs::JointControllerState::ConstPtr& MSGs) {

   	start_of_joint_pos(2) = MSGs->process_value;
}

void Get_value_wrist_1_joint(const control_msgs::JointControllerState::ConstPtr& MSGs) {

   	start_of_joint_pos(3) = MSGs->process_value;
}

void Get_value_wrist_2_joint(const control_msgs::JointControllerState::ConstPtr& MSGs) {

   	start_of_joint_pos(4) = MSGs->process_value;
}

void Get_value_wrist_3_joint(const control_msgs::JointControllerState::ConstPtr& MSGs) {

   	start_of_joint_pos(5) = MSGs->process_value;
}

float compute_linear(double q_start, double q_goal, float t, float max_time) {
	return((q_goal - q_start) * (t/max_time) + q_start);
}

void Get_ee_goal_pos(KDL::Frame ee_pos_start, KDL::Vector* ee_goal_pos_vector, float* max_time) {

	//ROS_INFO("Input the desired effector position offset:\n");
	std::cout << "Input the desired effector position offset:\n";

		//Get user input
		float x_pos,y_pos,z_pos;
		std::cout << "x_pos:";
		std::cin >> x_pos;
		std::cout << "y_pos:";
		std::cin >> y_pos;
		std::cout << "z_pos:";
		std::cin >> z_pos;
		std::cout << "Time of motion:";
		std::cin >> (*max_time);

		// ros::param::get("x_pos", x_pos);
		// ros::param::get("y_pos", y_pos);
		// ros::param::get("z_pos", z_pos);
		// ros::param::get("max_time", *max_time);

		//Compute goal position
		(*ee_goal_pos_vector)(0) = (ee_pos_start.p(0) + x_pos);
		(*ee_goal_pos_vector)(1) = (ee_pos_start.p(1) + y_pos);
		(*ee_goal_pos_vector)(2) = (ee_pos_start.p(2) + z_pos);
}

const int rate_of_loop = 100;

int main(int argc, char **argv)
{
	std::string urdf_path = ros::package::getPath("my-ur5-position-control");
	urdf_path += "/urdf/ur5_robot.urdf";
	ros::init(argc, argv, "my_ur5_pos_jnt_ctrl");

	ros::NodeHandle nh;

	ros::Rate loop_rate(rate_of_loop);
	std::cout << "Reached checkpoint 1";
	//Create subscribers for all joint states
	ros::Subscriber shoulder_pan_sub = nh.subscribe("/shoulder_pan_joint_position_controller/state", 1000, Get_value_shoulder_pan_joint);
	ros::Subscriber shoulder_lift_sub = nh.subscribe("/shoulder_lift_joint_position_controller/state", 1000, Get_value_shoulder_arm_joint);
	ros::Subscriber elbow_sub = nh.subscribe("/elbow_joint_position_controller/state", 1000, Get_value_elbow_joint);
	ros::Subscriber wrist_1_sub = nh.subscribe("/wrist_1_joint_position_controller/state", 1000, Get_value_wrist_1_joint);
	ros::Subscriber wrist_2_sub = nh.subscribe("/wrist_2_joint_position_controller/state", 1000, Get_value_wrist_2_joint);
	ros::Subscriber wrist_3_sub = nh.subscribe("/wrist_3_joint_position_controller/state", 1000, Get_value_wrist_3_joint);

	//Create publishers to send position commands to all joints
	ros::Publisher joint_parameter_pub[6]; 
	joint_parameter_pub[0] = nh.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
	joint_parameter_pub[1] = nh.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
	joint_parameter_pub[2] = nh.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
	joint_parameter_pub[3] = nh.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
	joint_parameter_pub[4] = nh.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
	joint_parameter_pub[5] = nh.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);


	//Parse urdf model and generate KDL tree
	KDL::Tree ur5_tree;
	if (!kdl_parser::treeFromFile(urdf_path, ur5_tree)){
		ROS_ERROR("Failed to construct kdl tree");
   		return false;
	}

	//Generate a kinematic chain from the robot base to its tcp
	KDL::Chain ur5_chain;
	ur5_tree.getChain("base_link", "wrist_3_link", ur5_chain);

	//Create solvers
	KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);
	KDL::ChainIkSolverVel_pinv vel_ik_solver(ur5_chain, 0.0001, 1000);
	KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver, 1000);

	//Make sure we have received proper joint angles already
	for(int i=0; i< 2; i++) {
		ros::spinOnce();
	 	loop_rate.sleep();
	}

	const float time_step = 1/((float)rate_of_loop);
	int count = 0;
	while (ros::ok()) {

		//Compute current tcp position
		KDL::Frame ee_pos_start;
		fk_solver.JntToCart(start_of_joint_pos	, ee_pos_start);

		ROS_INFO("Current tcp Position/Twist KDL:");		
		ROS_INFO("Position: %f %f %f", ee_pos_start.p(0), ee_pos_start.p(1), ee_pos_start.p(2));		
		ROS_INFO("Orientation: %f %f %f", ee_pos_start.M(0,0), ee_pos_start.M(1,0), ee_pos_start.M(2,0));

		//get user input
		float max_time;
		KDL::Vector ee_goal_pos_vector(0.0, 0.0, 0.0);
		Get_ee_goal_pos(ee_pos_start, &ee_goal_pos_vector, &max_time);

		KDL::Frame ee_goal_pos(ee_pos_start.M, ee_goal_pos_vector);

		//Compute inverse kinematics
		KDL::JntArray joint_goal_pos(num_of_joints);
		ik_solver.CartToJnt(start_of_joint_pos	, ee_goal_pos, joint_goal_pos);

		float t = 0.1;
		while(t<max_time) {
			std_msgs::Float64 position[6];
			//Compute next position step for all joints
			for(int i=0; i<num_of_joints; i++) {
				position[i].data = compute_linear(start_of_joint_pos(i),joint_goal_pos(i), t, max_time);//( start_of_joint_pos(i) - joint_goal_pos(i) )* (t/ max_time) + start_of_joint_pos(i) ; //linear interpolation
				joint_parameter_pub[i].publish(position[i]);
			}

			ros::spinOnce();
			loop_rate.sleep();
			++count;
			t += time_step;	
		}		
	}	
	return 0;
}
