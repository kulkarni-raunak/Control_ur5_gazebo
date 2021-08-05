# Control_ur5_gazebo
A small task to control joint angles of ur5 in application of force. 

All the data is in src folder and arranged according to different tasks.

## Note
1) Please install and build universal_robots from https://github.com/ros-industrial/universal_robot .
2) Clone/paste the contents of this src folder into your local catkin_ws/src folder.
3) Build the packages using catkin_make. 

## Task 1 - To swing the robot joints with sine function

 Use the following command
 
```
 roslaunch my-ur5-position-control just_pub_jnt_angles.launch
 ```
 
 ## Task 2 - Control the joint angles of ur5 to attempt a particular enf effector configuration on application on constant force on the later
 
 Use the following command
 
```roslaunch my-ur5-position-control gazebo_force.launch```
 
 Incase, ROS Logger does'nt ask for desired end_effector position use the follwing command in new terminal.
 
```rosrun my-ur5-position-control my_ur5_pos_jnt_ctrl```
 
 And feed in the values of x_pos, y_pos and z_pos.
 
 ## Future work
 
 As you saw, in second task you almost everytime have to execute the rosnode yourself which is tedious. I'll work shortly and pass the parameter that this node requires from user through launch files themselves.
