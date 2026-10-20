#pragma once 
#include <Eigen/Dense>
#include "nav_msgs/Odometry.h"      
#include "tf/tf.h" 
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

/**
 * @mainpage
 * This package contains an application of a Kalman Filter (KF) and an Extended Kalman Filter (EKF)
 * that works on the turtlebot3 robot. The launchfiles bringup_KF.launch and bringup_EKF.launch can 
 * be used to start the visualisation of the KF and EKF nodes respectively. 
 * **/

/**
* @brief The KF class applies a kalman filter to the noisy measured robot position and applied
* velocity to predict the current robot position.
* The measured position and applied velocity are read from the corresponding topics using "callback_odom" and 
* "callback_velocity_command" methods respectively. \n \n
* The position of the robot is illustrated in the state vector: \n
* <b> x_t = [x, y, theta]^T </b> \n
* Where theta is the yaw of the robot.
*
* The robot velocity vector is given by: \n
* <b> u = [x’, y’, theta’]^T = [v*cos(theta), v*sin(theta), omega]^T </b> \n
* where v is the linear velocity and omega is the angular velocity. \n
* 

* The change in the state vector is given by: \n
* <b> x_t = A*x_t-1 + B*u + G_u*r(0, R) </b> \n
* Where: \n <ul>
* <li> \b x_t-1: State vector at the time point (t-1) </li> \n
* <li> \b r(0, R): process noise illustrated as a normal distribution with a meanvalue of 0 and the covariance matrix R </li> \n
* <li> \b A: State-transition matrix \n
*   = \n
    [1,0,0] \n
    [0,1,0] \n
    [0,0,1] \n </li>
* <li> \b B: Control-Input matrix \n
*   = \n
    [dt,0 ,0 ] \n
    [0 ,dt,0 ] \n
    [0 ,0 ,dt] \n </li>
* <li> \b G_u: Process noise input matrix \n
    = \n
    [dt*cos(theta), 0] \n
    [dt*sin(theta), 0] \n
    [0            ,dt] \n </li>
* <li> \b R: Covariance of the process noise \n
*   = \n
    [0.001, 0     ,0    ] \n
    [0    , 0.001 ,0    ] \n
    [0    , 0     ,0.001] \n </li>
     </ul>
* 
* The measured position z_t is given by: \n
* <b> z_t = C*x_t + delta(0, Q) </b> \n
* where:  \n <ul>
  <li> \b delta(0, Q): a normal distribution with the meanvalue of 0 and the covariance Matrix Q \n  </li>
* <li> \b C: Observation matrix \n
*   = \n
    [1,0,0] \n
    [0,1,0] \n
    [0,0,1] \n </li>
* <li> \b Q: Covariance of the observation noise \n
*   = \n
    [0.001, 0     ,0    ] \n
    [0    , 0.001 ,0    ] \n
    [0    , 0     ,0.001] \n </li>
</ul>

* \b Note: the noise matrixes are set to zero when the robot is not moving, as the uncertainty in the position
comes from the robot movement. \n \n

* The Kalman filter is then evaluated as follows: \n
  <b> x_prediction = A*x_tminus1 + B*u </b> \n
  <b> Sigma_prediction = A*Sigma_t-1*A^T + G_u*R*G_u^T </b> \n
  <b> K = Sigma_prediction*C^T*(C*Sigma_prediction*C^T + Q)^-1 </b> \n
  where \b z is the measurement vector \b z = [x_meas, y_meas, theta_meas]^T \n
  <b> x_correction = x_prediction + K*(z - C*x_prediction) </b> \n
  <b> Sigma = (I - K*C)*Sigma_prediction </b> \n
*/

class KF
{
    public:
        /**
         * @brief Constructor of the Kalman filter class
        */
        KF();

        /**
         * @brief Callback function for the velocity commands. A subscriber should
         * subscribe to the velocity commands topic and use this function as a callback 
         * function to update the velocity commands in the kalman filter.
         * @param: Twist message that contains the velocity commands
        */
        void callback_velocity_command(const geometry_msgs::Twist::ConstPtr &msg);

        /**
         * @brief Callback function for the odomentry messages. A subscriber should
         * subscribe to the odometry topic and use this function as a callback 
         * function to update the measurements in the kalman filter.
         * @param: Odometry message that contains the measured robot position.
        */
        void callback_odom(const nav_msgs::Odometry::ConstPtr &msg);

        /**
         * @brief Prints the predicted value, measured value and covariance matrix of the robot position.
        */
        void printInfo();

        /**
         * @brief Updates the prediction and covariance matrix of the robot position. This function should be called
         * in constant time intervals (defined by dt) in order for the KF to work properly.
        */
        void update();

    private:

        Eigen::MatrixXd A,G_u,B,C,Sigma,R,Q,K,I;
        ros::Publisher odom_pub;
        tf::TransformBroadcaster odom_broadcaster;
        double dt;
        Eigen::VectorXd x_t_prediction, x_tminus1, x_t_correction, u, z, meas_velocities;
};