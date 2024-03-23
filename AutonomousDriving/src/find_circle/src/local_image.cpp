#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_publisher_node");
  ros::NodeHandle nh;

  // Create a publisher for the image
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/unity_ros/OurCar/Sensors/RGBCameraLeft/image_raw", 1);

  // Load the image from file
  cv::Mat image = cv::imread("/home/jididu/team15_cars/AutonomousDriving/15-print-circle-dots.jpg", cv::IMREAD_COLOR);

  // Check if the image was loaded successfully
  if (image.empty())
  {
    ROS_ERROR("Failed to load image");
    return -1;
  }

  // Create a ROS image message
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  // Set the timestamp of the image message (optional)
  msg->header.stamp = ros::Time::now();

  // Set the frame ID of the image message (optional)
  msg->header.frame_id = "your_frame_id";

  // Loop at a desired rate to publish the image
  ros::Rate rate(10); // 10 Hz
  while (ros::ok())
  {
    // Publish the image message
    image_pub.publish(msg);

    // Spin once to let ROS process the callbacks
    ros::spinOnce();

    // Sleep to maintain the desired publishing rate
    rate.sleep();
  }

  return 0;
}
