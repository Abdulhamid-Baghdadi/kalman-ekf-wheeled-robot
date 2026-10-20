#pragma once 
#include <Eigen/Dense>
#include "nav_msgs/Odometry.h"      
#include "tf/tf.h" 
#include <tf/transform_broadcaster.h>

/**
* @brief The EKF class applies an extended kalman filter to the noisy measured robot position and applied
* velocity to predict the current robot position.
* The measured position and applied velocity are read from the corresponding topics using "callback_odom" and 
* "callback_velocity_command" methods respectively. \n \n
* The position of the robot is illustrated in the state vector: \n
* <b> x_t = \n
    [x] \n
    [y] \n
    [theta] \n
 </b> \n
* Where theta is the yaw of the robot.
*
* The robot velocity vector is given by: \n
* <b> u =  [v, omega]^T </b> \n
* where v is the linear velocity and omega is the angular velocity. \n
* 
* The change in the state vector is given by: \n
* <b> x_t = g(x_t-1, u_t-1) + G_u*r(0,R) \n
 </b> \n
* where: \n <ul>
* <li> \b g(x_t-1, u_t-1) = \n
        [x_t-1 + dt*v*cos(theta_t-1)] \n
        [y_t-1 + dt*v*sin(theta_t-1)] \n
        [theta_t-1 + dt*omega] \n </li> \n 
* <li> \b x_t-1: State vector at the time point (t-1) </li> \n
* <li> \b r(0, R): process noise illustrated as a normal distribution with a meanvalue of 0 and the covariance matrix R </li> \n
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
* <b> z_t = H*x_t + delta(0, Q) </b> \n
* where:  \n <ul>
  <li> \b delta(0, Q): a normal distribution with the meanvalue of 0 and the covariance Matrix Q \n  </li>
* <li> \b H: Observation matrix \n
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
  <b> x_prediction = g(x_t-1, u_t) </b> \n
  <b> Sigma_prediction = G_t*Sigma_t-1*G_t^T + G_u*R*G_u^T </b> \n
  * where: \n
  <b> G_t =  jacobian (g(x_t-1, u_t), x_t) </b> =  \n
    [1, 0, -dt*v*sin(theta_t-1)] \n
    [0, 1,  dt*v*cos(theta_t-1)] \n
    [0, 0,  0                  ] \n
  <b> K = Sigma_prediction*H^T*(H*Sigma_prediction*H^T + Q)^-1 </b> \n
  where \b z is the measurement vector \b z = [x_meas, y_meas, theta_meas]^T \n
  <b> x_correction = x_prediction + K*(z - H*x_prediction) </b> \n
  <b> Sigma = (I - K*H)*Sigma_prediction </b> \n
*/

class EKF
{
    public:
        /**
         * @brief Constructor of the extended Kalman filter class
        */
        EKF();

        /**
         * @brief Callback function for the velocity commands. A subscriber should
         * subscribe to the velocity commands topic and use this function as a callback 
         * function to update the velocity commands in the extended kalman filter.
         * @param: Twist message that contains the velocity commands
        */
        void callback_velocity_command(const geometry_msgs::Twist::ConstPtr &msg);

        /**
         * @brief Callback function for the odomentry messages. A subscriber should
         * subscribe to the odometry topic and use this function as a callback 
         * function to update the measurements in the extended kalman filter.
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

        Eigen::MatrixXd G_x, G_u,Sigma,R,H,Q,K,I;
        ros::Publisher odom_pub;
        tf::TransformBroadcaster odom_broadcaster;        
        double dt;
        bool initialized;
        Eigen::VectorXd x_t_prediction, x_tminus1, x_t_correction, u, z, meas_velocities;
        
};