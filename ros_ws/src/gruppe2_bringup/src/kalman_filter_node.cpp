#include <Eigen/Dense>
#include "geometry_msgs/Pose.h"     
#include "nav_msgs/Odometry.h"      
#include "ros/ros.h"                
#include "std_msgs/String.h"        
#include <ros/console.h>
#include <iostream>
#include "tf/tf.h" 
#include "KF.hpp"
#include "sensor_msgs/JointState.h"

/**
 * @brief Starts the Kalman filter and subscribes to the odometry and velocity control commands topics.
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "kalman_filter_node");                                           
  
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_data;
    ros::Subscriber odom_data;

    KF kf;

    cmd_vel_data = nh.subscribe("/rosbot/cmd_vel", 1, &KF::callback_velocity_command, &kf);
    odom_data = nh.subscribe("/rosbot/odom", 1, &KF::callback_odom, &kf);
        
    while(ros::ok)                                                              
    {   
        ros::spinOnce();
    }

    return 0;
}
