// Gazebo dependencies
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo/physics/Model.hh>

// ROS dependencies
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <iostream>
#include <stdio.h>

double ros_rate=100;

int main(int argc, char **argv) {
    ros::init(argc, argv, "apply_force_node");
    ros::NodeHandle nh;

    ros::ServiceClient wrenchClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
    gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;

    // gazebo::physics::Model GetBody(const std::string &name);

    //std::cout<<name<<std::endl;

    bool service_ready = false;
    while (!service_ready) {
          service_ready = ros::service::exists("/gazebo/apply_body_wrench", true);
         // ROS_INFO("waiting for apply_body_wrench service");
          ros::Duration(0.5).sleep();
    }
    //ROS_INFO("apply_body_wrench service is ready");

    ros::Time time_temp(0, 0);
    ros::Duration duration_temp(0, 1000000);
    apply_wrench_req.wrench.force.x = 0.0;
    apply_wrench_req.wrench.force.y = 0.0;
    apply_wrench_req.wrench.force.z = 200.0;
    apply_wrench_req.start_time = time_temp;
    apply_wrench_req.duration = duration_temp;
    apply_wrench_req.body_name = "wrist_3_link";
    apply_wrench_req.reference_frame = "world";
    ros::Rate loop_rate(ros_rate);
    ros::console::shutdown();
    while (ros::ok())
    {
        wrenchClient.call(apply_wrench_req, apply_wrench_resp);
        ros::Duration(0.5).sleep();
    }
    
    

     return EXIT_SUCCESS;;
}
