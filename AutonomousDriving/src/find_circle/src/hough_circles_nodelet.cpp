#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    // Convert ROS image message to OpenCV format
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    // Convert image to grayscale
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    // Apply Hough Circle Transform
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(grayImage, circles, cv::HOUGH_GRADIENT, 1,
                     grayImage.rows / 8, 120, 80, 0, 0);

    // Draw detected circles on the original image
    for (size_t i = 0; i < circles.size(); i++)
    {
      cv::Vec3f circle = circles[i];
      cv::Point center(circle[0], circle[1]);
      int radius = circle[2];
      cv::circle(image, center, radius, cv::Scalar(0, 255, 0), 2);
    }

    // Display the image with detected circles
    cv::imshow("Detected Circles", image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "circle_detection_node");
  ros::NodeHandle nh;

  // Subscribe to the RGB image topic
  ros::Subscriber sub = nh.subscribe("/unity_ros/OurCar/Sensors/RGBCameraLeft/image_raw", 1, imageCallback);

  ros::Rate rate(10); // 10 Hz

  // OpenCV GUI window
  cv::namedWindow("Detected Circles");

  while (ros::ok())
  {
    ros::spinOnce(); // Process any pending ROS messages

    // Perform circle detection

    rate.sleep(); // Sleep to achieve the desired frequency
  }

  cv::destroyWindow("Detected Circles");

  return 0;
}
