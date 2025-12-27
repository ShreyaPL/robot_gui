#include "robot_gui/robot_gui_node.h"

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

RobotGuiNode::RobotGuiNode(ros::NodeHandle &nh)
    :n(nh),
    robot_info_text("(no /robot_info received yet)"),
    frame(cv::Size(900, 600), CV_8UC3),
    window_name("Robot Info GUI"){
    
    sub_robot_info = n.subscribe("/robot_info", 10, &RobotGuiNode::robotInfoCb, this);

    // Init CVUI
    cvui::init(window_name.c_str());
    }

void RobotGuiNode::robotInfoCb(const std_msgs::String::ConstPtr &msg){
    robot_info_text = msg->data;
}

void RobotGuiNode::run() {
    ros::Rate rate(30);
    while (ros::ok()){
        ros::spinOnce();

        frame = cv::Scalar(49, 52, 49);

        //header
        cvui::text(frame, 20, 15, "Robot GUI", 0.6, 0xFFFFFF);

        //General Info Area
        cvui::window(frame, 20, 50, 420, 200, "General Info (/robot_info)");
        cvui::printf(frame, 35, 85, 0.45, 0xFFFFFF, "%s", robot_info_text.c_str());

        cvui::update();
        cvui::imshow(window_name, frame);
        if (cv::waitKey(1) == 27) break;

        rate.sleep();
    }
}