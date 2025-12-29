#include "robot_gui/robot_gui_node.h"
#include "geometry_msgs/Twist.h"

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

RobotGuiNode::RobotGuiNode(ros::NodeHandle &nh)
    :n(nh),
    robot_info_text("(no /robot_info received yet)"),
    frame(cv::Size(600, 900), CV_8UC3),
    window_name("Robot Info GUI"),
    lin_step(0.05),
    ang_step(0.10),
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
        cvui::window(frame, 20, 280, 500, 170, "Teleoperation (/cooper_1/cmd_vel)");

        //button sizes
        const int bw = 110;
        const int bh = 45;

        // center anchor
        const int cx = 20 + 560 / 2;
        const int cy = 240 + 240 / 2; 

        //Forward
        if (cvui::button(frame, cx - bw/2, cy - (bh+10), bw, bh, "Forward")) {
            target_lin_x += lin_step;
        }

        //Back
        if (cvui::button(frame, cx - bw/2, cy + 10, bw, bh, "Back")) {
            target_lin_x -= lin_step;
        }

        // Left
        if (cvui::button(frame, cx - (bw + 10), cy - bh/2, bw, bh, "Left")) {
            target_ang_z += ang_step;
        }

        //Right
        if (cvui::button(frame, cx + 10, cy - bh/2, bw, bh, "Right")) {
            target_ang_z -= ang_step;
        }

        //Stop
        if (cvui::button(frame, cx - bw/2, cy - bh/2, bw, bh, "STOP")) {
            target_lin_x = 0.0;
            target_ang_z = 0.0;
        }

        cvui::printf(frame, 40, 465, 0.50, 0xFFFFFF, "Commanding: lin.x=%.2f  ang.z=%.2f", target_lin_x, target_ang_z);

        //Publish continuously
        geometry_msgs::Twist cmd;
        cmd.linear.x = target_lin_x;
        cmd.angular.z = target_ang_z;
        pub_cmd_vel.publish(cmd);

        //Render
        cvui::update();
        cv::imshow(window_name, frame);
        if (cv::waitKey(1) == 27) break;

        rate.sleep();
    }
}