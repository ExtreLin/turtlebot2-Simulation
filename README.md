# turtlebot2-Simulation

## Depends
1. ROS
2. turtlebot_gazebo turtlebot_world.launch 
3. turtlebot_gazebo amcl_demo.launch 
4. OpenCV with cuda 
5. Qt 5.13 or highter 
6. Boost 
7. Cuda 9.0 or highter 
8. Eigen 3.0 
9. OpenMesh 
10. glog 
11. ceres 

## compile
1. cmake 
2. make -j14

or you can use vscode with cmake extension.

## How to Use
1. new console input "roslaunch turtlebot_gazebo turtlebot_world.launch" 
2. new console input "roslaunch turtlebot_gazebo amcl_demo.launch" 
3. new console input "Sn3DCarScan_node" 

## Algorithm
1. using "kinect fusion lib " to rebuild. fork https://github.com/chrdiller/KinectFusionLib
2. using "robust point cloud registration" to registration two rangdata. fork https://github.com/ethz-asl/robust_point_cloud_registration

## Others
1. render the rebuilded mesh by vulkan.
