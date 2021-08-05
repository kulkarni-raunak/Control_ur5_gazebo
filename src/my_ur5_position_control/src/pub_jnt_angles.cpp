#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"

#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <time.h>

const double pi = 3.14159265358979323846;


const int loop_rate_val = 20;


int main(int argc, char **argv)
{
	std::string urdf_path = ros::package::getPath("my-ur5-position-control");
	if(urdf_path.empty()) {
		ROS_ERROR("my-ur5-position-control package path was not found");
	}
	urdf_path += "/urdf/ur5_robot.urdf";
	ros::init(argc, argv, "pub_jnt_angles");

	time_t timer;

	ros::NodeHandle n;

	ros::Rate loop_rate(loop_rate_val);

	//Create publishers to send position commands to all joints
	ros::Publisher joint_com_pub[6]; 
	joint_com_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
	joint_com_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
	joint_com_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
	joint_com_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
	joint_com_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
	joint_com_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);

	//Make sure we have received proper joint angles already
	for(int i=0; i< 2; i++) {
		ros::spinOnce();
	 	loop_rate.sleep();
	}
	
	float t = 1/((float)loop_rate_val);

	while (ros::ok()) {

		std_msgs::Float64 position[6];
		//publish to all joints with sine function
		for (int k = 0; k < 6; k++)
		{
			for (int i = 0; i < 2*pi; i++){
			position[k].data = sin(i);
			joint_com_pub[k].publish(position[k]);
			loop_rate.sleep();
			}
		}

		ros::spinOnce();
		loop_rate.sleep();

	}	
	return 0;
}
