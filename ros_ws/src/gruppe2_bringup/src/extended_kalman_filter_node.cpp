#include <Eigen/Dense>
#include "geometry_msgs/Pose.h"     
#include "nav_msgs/Odometry.h"      
#include "ros/ros.h"                
#include "std_msgs/String.h"        
#include <ros/console.h>
#include <iostream>
#include "tf/tf.h" 
#include "EKF.hpp"


/**
 * @brief Starts the extended Kalman filter and subscribes to the odometry and velocity control commands topics.
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "extended_kalman_filter_node");                                           
  
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_data;
    ros::Subscriber odom_data;

    EKF ekf;

    cmd_vel_data = nh.subscribe("/rosbot/cmd_vel", 1, &EKF::callback_velocity_command, &ekf);
    odom_data = nh.subscribe("/rosbot/odom", 1, &EKF::callback_odom, &ekf);
    
    while(ros::ok)                                                              
    {   
        ros::spin();   
    }

    return 0;
}
