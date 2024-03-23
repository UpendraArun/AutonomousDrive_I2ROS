#include <ros/ros.h>

#include <ros/console.h>

#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define PI M_PI

#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

#include "trajectory_follower.h"

class controllerNode
{
public:
  controllerNode() : hz(1000.0)
  {
    current_state_ = nh_.subscribe("current_state_est", 1, &controllerNode::onCurrentState, this);
    desired_pose_ = nh_.subscribe("pose_est", 1, &controllerNode::onDesiredPose, this);
    desired_twist_ = nh_.subscribe("twist_est", 1, &controllerNode::onDesiredTwist, this);
    ackermann_cmds_ = nh_.subscribe("cmd_multiplier_node/ackermann_cmd_mux", 1, &controllerNode::AckermannDriveStamped, this);
    car_commands_ = nh_.advertise<mav_msgs::Actuators>("car_commands", 1);
    timer_ = nh_.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
  }

  void onCurrentState(const nav_msgs::Odometry &cur_state)
  {
    x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y,
        cur_state.pose.pose.position.z;
    v << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y,
        cur_state.twist.twist.linear.z;
    omega << cur_state.twist.twist.angular.x, cur_state.twist.twist.angular.y,
        cur_state.twist.twist.angular.z;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(cur_state.pose.pose.orientation, q);
    R = q.toRotationMatrix();

    // Rotate omega
    omega = R.transpose() * omega;
  }

  void onDesiredPose(const geometry_msgs::PoseStamped &desired_pose)
  {
    xd << desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z;
    Eigen::Quaterniond q;
    q.x() = desired_pose.pose.orientation.x;
    q.y() = desired_pose.pose.orientation.y;
    q.z() = desired_pose.pose.orientation.z;
    q.w() = desired_pose.pose.orientation.w;

    yawd = std::atan2((2 * (q.w() * q.z() + q.x() * q.x())), (1 - 2 * (q.x() * q.x() + q.z() * q.z())));
  }

  void onDesiredTwist(const geometry_msgs::TwistStamped &desired_twist)
  {
    vd << desired_twist.twist.linear.x, desired_twist.twist.linear.y, desired_twist.twist.linear.z;
  }

  void AckermannDriveStamped(const ackermann_msgs::AckermannDriveStamped &ackermann)
  {
    if (acceleration == 0)
    {
      acceleration = 1;
    }

    Control::PID vel_pid = Control::PID(0.01, max_aceleration, min_aceleration, 0.1, 0.1, 0.1);
    acceleration = vel_pid.calculate(ackermann.drive.speed, v(0));

    Control::PID yawd_pid = Control::PID(0.01, max_turning_angle, min_turning_angle, 0.3, 0.1, 0.1);

    yawd = vel_pid.calculate(ackermann.drive.steering_angle, omega(2));
  }

  void controlLoop(const ros::TimerEvent &t)
  {
    mav_msgs::Actuators msg;

    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = acceleration; // Acceleration
    msg.angular_velocities[1] = yawd;         // Turning angle
    msg.angular_velocities[2] = 0;            // Breaking
    msg.angular_velocities[3] = 0;

    car_commands_.publish(msg);
  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber current_state_;
  ros::Subscriber desired_pose_;
  ros::Subscriber desired_twist_;
  ros::Subscriber ackermann_cmds_;
  ros::Publisher car_commands_;
  ros::Timer timer_;

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd; // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd; // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad; // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;        // desired yaw angle
  double acceleration;

  double max_aceleration{8.0};
  double min_aceleration{8.0};
  double max_turning_angle{2.0};
  double min_turning_angle{2.0};

  double hz; // frequency of the main control loop
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;

  ros::spin();
}
