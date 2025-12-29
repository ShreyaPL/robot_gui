#pragma once

#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

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
    void cmdVelCb(const geometry_msgs::Twist::ConstPtr & msg);                                                                                                                                                                                                                      

    //helpers
    static std::string fmtDouble(double v, int prec = 2);

    //ROS
    ros::NodeHandle n;
    ros::Subscriber sub_robot_info;
    ros::Subscriber sub_cmd_vel;

    ros::Publisher pub_cmd_vel;

    //State for display
    std::string robot_info_text;
    bool have_cmd_vel;
    geometry_msgs::Twist last_cmd_vel;
    double lin_step;
    double ang_step;

    //Command Velocities
    double target_lin_x;
    double target_ang_z;

    int publish_hz;

    //GUI
    cv::Mat frame;
    std::string window_name;
};