#include "EKF.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"     


EKF::EKF() {
    dt = 0.02; // time interval between function calls
    G_x =  Eigen::MatrixXd::Zero(3, 3);
    G_u =  Eigen::MatrixXd::Zero(3, 2);
    Sigma =  Eigen::MatrixXd::Zero(3, 3);
    R =  Eigen::MatrixXd::Zero(2, 2);
    H =  Eigen::MatrixXd::Identity(3, 3);
    Q =  Eigen::MatrixXd::Identity(3, 3) * 0.01;
    K =  Eigen::MatrixXd::Zero(3, 3);
    I =  Eigen::MatrixXd::Identity(3, 3);

    z = Eigen::VectorXd(3);
    u = Eigen::VectorXd(2);
    x_t_prediction = Eigen::VectorXd(3);
    x_tminus1 = Eigen::VectorXd(3);
    x_t_correction = Eigen::VectorXd(3);
    meas_velocities = Eigen::VectorXd(2);
    x_tminus1  << -2, 1, 0; //Starting pos
    x_t_correction  << -2, 1, 0; //Starting pos

    ros::NodeHandle n;
    odom_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("EKF_Position", 10);
}


void EKF::callback_velocity_command(const geometry_msgs::Twist::ConstPtr &msg)
{
    double v =  msg->linear.x;
    double omega = msg->angular.z;
    u << v, omega; 
}

void EKF::callback_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
    double roll, pitch, yaw;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    //Saves Odom data in z .... (x,x',y,yaw,yaw') yaw is converted from quaternion
    z <<    msg->pose.pose.position.x, 
            msg->pose.pose.position.y,
            yaw;           
    meas_velocities << msg->twist.twist.linear.x, msg->twist.twist.angular.z; 
    update();   
}

void EKF::printInfo()
{
    std::cout <<"EKF Prediction:" <<
                "\nx : " << x_t_correction(0) << 
                "\ny : " << x_t_correction(1) <<
                "\nYaw: " << x_t_correction(2) <<
                std::endl;
    std::cout << "Measured position:\n" << z  << std::endl;
    std::cout << "Covariance:\n" << Sigma  << std::endl;
}

void EKF::update()
{
    if(abs(meas_velocities(0)) > 0.001 || abs(meas_velocities(1)) > 0.001) {
        G_x << 1, 0, -dt*u(0)*sin(x_tminus1(2)),
            0, 1, dt*u(0)*cos(x_tminus1(2)),
            0, 0, 1;

        G_u <<  dt*cos(x_tminus1(2)), 0,
                dt*sin(x_tminus1(2)), 0,
                0, dt;

        R << 0.01*(1 ? abs(meas_velocities(0)) > 0.001 : 0), 0,
                0                  , 0.01*(1 ? abs(meas_velocities(1)) > 0.001 : 0);

        x_t_prediction << x_t_correction(0) + dt*u(0)*cos(x_t_correction(2)),
                        x_t_correction(1) + dt*u(0)*sin(x_t_correction(2)),
                        x_t_correction(2) + dt*u(1);     
        
        Eigen::MatrixXd Sigma_prediction = G_x*Sigma*G_x.transpose() + G_u*R*G_u.transpose();
        K = Sigma_prediction * H.transpose() * (H * Sigma_prediction * H.transpose() + Q).inverse();
        x_t_correction = x_t_prediction + K * (z - x_t_prediction);
        x_tminus1 = x_t_correction;
        Sigma = (I - K * H) * Sigma_prediction; 
        printInfo();
    }


    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(x_t_correction[2]);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "EKF_Position";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x_t_correction[0];
    odom_trans.transform.translation.y = x_t_correction[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "EKF_Position";
    msg.pose.pose.position.x = x_t_correction[0];
    msg.pose.pose.position.y = x_t_correction[1];
    msg.pose.pose.position.z = 0;

    tf::Quaternion q;
    q.setRPY(0,0,x_t_correction[2]);
    msg.pose.pose.orientation.x = q[0];
    msg.pose.pose.orientation.y = q[1];
    msg.pose.pose.orientation.z = q[2];
    msg.pose.pose.orientation.w = q[3];
    msg.pose.covariance = { Sigma(0,0), Sigma(0,1), 0, 0, 0, 0,
                            Sigma(1,0), Sigma(1,1), 0, 0, 0, 0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,Sigma(2,2)};
    odom_pub.publish(msg);
}