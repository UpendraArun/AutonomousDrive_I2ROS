#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_navigation_goals");

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Read waypoints points from YAML file
    std::string yaml_file = "~config/waypoints.yaml";
    YAML::Node yaml_node = YAML::LoadFile(yaml_file);

    if (!yaml_node.IsSequence())
    {
        ROS_ERROR("Invalid YAML file format. Expected a sequence of coordinates.");
        return 1;
    }

    for (const auto &point : yaml_node)
    {
        if (!point["x"] || !point["y"] || !point["w"])
        {
            ROS_ERROR("Invalid coordinate format. Each coordinate should have 'x', 'y', and 'w' fields.");
            return 1;
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = point["x"].as<double>();
        goal.target_pose.pose.position.y = point["y"].as<double>();
        goal.target_pose.pose.orientation.w = point["w"].as<double>();

        ROS_INFO("Sending move base goal");
        ac.sendGoal(goal);
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Robot has arrived at the goal position");
        else
            ROS_INFO("The base failed for some reason");
    }

    return 0;
}
