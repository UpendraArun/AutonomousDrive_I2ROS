#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <cmath>

ros::Publisher pub;
double wheelbase;
std::string ackermann_cmd_topic;
std::string frame_id;
// Function to convert linear and angular velocities to steering angle.

double convert_trans_rot_vel_to_steering_angle(double v, double omega, double wheelbase)
{
    if (omega == 0 || v == 0)
    {
        return 0;
    }

    double radius = v / omega;
    return std::atan(wheelbase / radius);
}

// Callback function to process twist commands and generate Ackermann drive commands.

void cmdCallback(const geometry_msgs::Twist::ConstPtr &data)
{
    double v = data->linear.x;
    double steering = convert_trans_rot_vel_to_steering_angle(v, data->angular.z, wheelbase);

    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.drive.steering_angle = steering;
    msg.drive.speed = v;

    pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_to_ackermann_drive");
    ros::NodeHandle nh("~");

    std::string twist_cmd_topic;
    nh.param<std::string>("twist_cmd_topic", twist_cmd_topic, "/cmd_vel");
    nh.param<std::string>("ackermann_cmd_topic", ackermann_cmd_topic, "/ackermann_cmd");
    nh.param<double>("wheelbase", wheelbase, 1.0);
    nh.param<std::string>("frame_id", frame_id, "odom");
    // Subscribe to the twist topic and advertise the Ackermann drive topic.
    ros::Subscriber sub = nh.subscribe(twist_cmd_topic, 1, cmdCallback);
    pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>(ackermann_cmd_topic, 1);

    ROS_INFO("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f",
             twist_cmd_topic.c_str(), ackermann_cmd_topic.c_str(), frame_id.c_str(), wheelbase);

    ros::spin();

    return 0;
}
