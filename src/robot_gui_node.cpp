#include "robot_gui/robot_gui_node.h"
#include "geometry_msgs/Twist.h"

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

RobotGuiNode::RobotGuiNode(ros::NodeHandle &nh)
    :n(nh),
    robot_info_text("(no /robot_info received yet)"),
    frame(cv::Size(600, 900), CV_8UC3),
    window_name("Robot Info GUI"),
    target_lin_x(0.0),
    target_ang_z(0.0),
    have_cmd_vel(false),
    publish_hz(30){
    
    sub_robot_info = n.subscribe("/robot_info", 10, &RobotGuiNode::robotInfoCb, this);
    sub_cmd_vel = n.subscribe("/cooper_1/cmd_vel", 10, &RobotGuiNode::cmdVelCb, this);
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cooper_1/cmd_vel", 10);

    // Init CVUI
    cvui::init(window_name.c_str());
    }

void RobotGuiNode::robotInfoCb(const std_msgs::String::ConstPtr &msg){
    robot_info_text = msg->data;
}

void RobotGuiNode::cmdVelCb(const geometry_msgs::Twist::ConstPtr &msg){
    last_cmd_vel = *msg;
    have_cmd_vel = true;
}

void RobotGuiNode::run() {
    ros::Rate rate(publish_hz);
    while (ros::ok()){
        ros::spinOnce();

        frame = cv::Scalar(49, 52, 49);

        //header
        cvui::text(frame, 20, 15, "Robot GUI", 0.6, 0xFFFFFF);

        //General Info Area
        cvui::window(frame, 20, 40, 380, 180, "General Info (/robot_info)");
        cvui::printf(frame, 35, 85, 0.45, 0xFFFFFF, "%s", robot_info_text.c_str());

        //Teleop Buttons
        cvui::window(frame, 20, 240, 380, 220, "Teleoperation (/cooper_1/cmd_vel)");
        if (cvui::button(frame, 40, 330, 120, 40, "STOP")) {
            target_lin_x = 0.0;
            target_ang_z = 0.0;
        }

        // double shown_lin = have_cmd_vel ? last_cmd_vel.linear.x : target_lin_x;
        // double shown_ang = have_cmd_vel ? last_cmd_vel.angular.z : target_ang_z;
        // cvui::printf(frame, 180, 342, 0.45, 0xFFFFFF, "Commanding: lin.x=%.2f   ang.z=%.2f", shown_lin, shown_ang);

        //Render
        cvui::update();
        cvui::imshow(window_name, frame);
        if (cv::waitKey(1) == 27) break;

        geometry_msgs::Twist cmd;
        cmd.linear.x = target_lin_x;
        cmd.angular.z = target_ang_z;
        pub_cmd_vel.publish(cmd);

        rate.sleep();
    }
}