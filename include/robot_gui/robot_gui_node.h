#pragma once

#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <opencv2/opencv.hpp>
// #include <robot_gui/cvui.h>
#include <string>

class RobotGuiNode{

public:
    RobotGuiNode(ros::NodeHandle &nh);
    void run();

private:
    //callbacks
    void robotInfoCb(const std_msgs::String::ConstPtr & msg);

    //helpers
    static std::string fmtDouble(double v, int prec = 2);

    //ROS
    ros::NodeHandle n;
    ros::Subscriber sub_robot_info;

    //State for display
    std::string robot_info_text;

    //GUI
    cv::Mat frame;
    std::string window_name;
};