#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
// opencv stuffs
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class image_processing
{
  // Node
  ros::NodeHandle nh_;
  ros::Timer timer;

  // Initialization ros stuffs
  ros::Subscriber depth_sub;
  ros::Subscriber rgb_sub;
  ros::Subscriber semantic_sub;
  // ros::Publisher traffic_light_pub;
  ros::Publisher tl_distance_pub;
  ros::Publisher pub;
  std::string ackermann_cmd_stop;
  std::string frame_id;
  sensor_msgs::Image depth_image;
  sensor_msgs::Image rgb_image;
  sensor_msgs::Image semantic_image;
  bool isRed;
  std_msgs::Int32 msg_tl_distance;

  // Initial CV and CV Bridge stuffs
  cv_bridge::CvImagePtr cv_ptr1;
  cv_bridge::CvImagePtr cv_ptr2;
  cv_bridge::CvImagePtr cv_ptr3;
  cv::Mat depth;
  cv::Mat depth_seg;
  cv::Mat rgb;
  cv::Mat semantic;
  cv::Mat semantic_grey;
  cv::Mat semantic_grey2;
  cv::Mat rgb_seg2;
  cv::Mat rgb_seg;

  double int_distance_tl = 255;

public:
  // Constructor
  image_processing()
  {
    depth_sub = nh_.subscribe("/realsense/depth/image", 100,
                              &image_processing::callback_depth, this);
    rgb_sub = nh_.subscribe("/realsense/rgb/left_image_raw", 100,
                            &image_processing::callback_rgb, this);
    semantic_sub = nh_.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 100,
                                 &image_processing::callback_semantic, this);

    tl_distance_pub = nh_.advertise<std_msgs::Int32>("tl_distance", 1);

    pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("stop_ackermann", 1);

    timer = nh_.createTimer(ros::Duration(0.1), &image_processing::mainLoop, this);
  }

  //////// MAIN LOOP ////////
  void mainLoop(const ros::TimerEvent &t)
  {
    // Only if all images are received
    if ((depth.cols != 0) && (rgb.cols != 0) && (semantic.cols != 0))
    {
      int_distance_tl = 255;

      // Assuming you have the 'semantic' image already loaded in cv::Mat format
      cv::Mat mask = cv::Mat::zeros(semantic.size(), CV_8UC1);

      // Define the region corresponding to the right-hand side
      int right_width = semantic.cols * 2 / 3; // Width of the right 2/3 region

      // Set the values in the mask to 1 for the right-hand side region
      mask(cv::Rect(semantic.cols - right_width, 0, right_width, semantic.rows)) = 1;
      cv::Mat semantic_masked;
      semantic.copyTo(semantic_masked, mask);

      // Find the area where traffic light is and then crop it out
      cvtColor(semantic_masked, semantic_grey, cv::COLOR_BGR2GRAY);
      cv::threshold(semantic_grey, semantic_grey2, 210, 255, cv::THRESH_BINARY);
      semantic_grey2 = semantic_grey2 / 255;

      cvtColor(semantic_grey2, semantic_grey, cv::COLOR_GRAY2RGB);
      rgb_seg = semantic_grey.mul(rgb);
      semantic_grey2.convertTo(semantic_grey, CV_16UC1);
      depth_seg = semantic_grey.mul(depth);
      // initialize the traffic light counter
      int count_red = 0;
      int count_green = 0;
      // check the pixel value of the traffic light area
      for (int i = 1; i < rgb_seg.rows; i++)
      {
        for (int j = 1; j < rgb_seg.cols; j++)
        {

          if ((((int)rgb_seg.at<cv::Vec3b>(i, j)[2]) - ((int)rgb_seg.at<cv::Vec3b>(i, j)[1])) > 110)
          {
            count_red++;
            ROS_INFO("RED_DIS= %d ", (int)depth_seg.at<ushort>(i, j));
            int_distance_tl = ((int_distance_tl * (double)(count_red - 1) + (double)depth_seg.at<ushort>(i, j)) / (double)count_red); // TODO
            ROS_INFO("RED_DIS_INT= %f ", int_distance_tl);
          }
          else if ((((int)rgb_seg.at<cv::Vec3b>(i, j)[1]) - ((int)rgb_seg.at<cv::Vec3b>(i, j)[2])) > 150)
          {
            count_green++;
            ROS_INFO("GREEN_DIS= %d ", (int)depth_seg.at<ushort>(i, j));
          }
        }
      }

      // change the flag msg
      if (count_green > 5 && count_green > count_red)
      {
        isRed = false;
        ROS_INFO("green! red_count: %d   green_count: %d", count_red, count_green);
      }
      else if (count_red > 1 && count_red > count_green)
      {
        isRed = true;
        ROS_INFO("red! red_count: %d   green_count: %d", count_red, count_green);
        ROS_INFO("Distance = %f", int_distance_tl);
      }
      else
      {
        isRed = false;
        ROS_INFO("No Red light detected \n");
      }
      msg_tl_distance.data = (int)int_distance_tl;

      // publish msgs

      tl_distance_pub.publish(msg_tl_distance);

      if (int_distance_tl < 20000 && isRed)
      {
        ackermann_msgs::AckermannDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id;
        msg.drive.steering_angle = 0.0;
        msg.drive.speed = 0;
        pub.publish(msg);
      }
      else
      {
        ackermann_msgs::AckermannDriveStamped msg;
        msg.header.frame_id = frame_id;
        msg.drive.steering_angle = 1.0;
        msg.drive.speed = 1;
        pub.publish(msg);
      }
    }
  };

  //////// CALLBACK FUNCTION /////////

  void callback_depth(const sensor_msgs::Image &img)
  {
    cv_ptr1 = cv_bridge::toCvCopy(img, "16UC1");
    depth = cv_ptr1->image;
    cv::imwrite("depth.jpg", depth);
  }
  void callback_rgb(const sensor_msgs::Image &img)
  {
    cv_ptr2 = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    rgb = cv_ptr2->image;
    cv::imwrite("rgb.jpg", rgb);
  }
  void callback_semantic(const sensor_msgs::Image &img)
  {
    cv_ptr3 = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    semantic = cv_ptr3->image;
    cv::imwrite("semantic.jpg", semantic);
  }
};

// Main function
int main(int argc, char **argv)
{
  // initialize node
  ros::init(argc, argv, "image_processing");

  ROS_INFO("sensoring seccessfully created");

  // initialize class
  image_processing image_processing;

  // Allow the timer to be called
  ros::spin();
}