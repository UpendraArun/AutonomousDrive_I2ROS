#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class MultiplierNode
{
public:
    MultiplierNode() : nh("~")
    {
        // Read topic names from parameters
        nh.param<std::string>("topic1", topic1, "/ackermann_cmd");
        nh.param<std::string>("topic2", topic2, "/stop_ackermann");

        // Subscribe to the input topics
        sub1 = nh.subscribe(topic1, 1, &MultiplierNode::callbackTopic1, this);
        sub2 = nh.subscribe(topic2, 1, &MultiplierNode::callbackTopic2, this);

        // Advertise the output topic
        pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd_mux", 1);
    }

    void callbackTopic1(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg)
    {
        msg1 = *msg;
        publishOutput();
    }

    void callbackTopic2(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg)
    {
        msg2 = *msg;
        publishOutput();
    }

    void publishOutput()
    {
        // Multiply the corresponding values from both messages
        ackermann_msgs::AckermannDriveStamped output_msg;
        output_msg.header.frame_id = frame_id;
        output_msg.drive.steering_angle = msg1.drive.steering_angle * msg2.drive.steering_angle;
        output_msg.drive.speed = msg1.drive.speed * msg2.drive.speed;

        // Publish the output message
        pub.publish(output_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Publisher pub;
    ackermann_msgs::AckermannDriveStamped msg1;
    ackermann_msgs::AckermannDriveStamped msg2;
    std::string topic1, topic2;
    std::string frame_id = "base_link"; // Set the appropriate frame ID here
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multiplier_node");
    MultiplierNode multiplier;
    ros::spin();
    return 0;
}
