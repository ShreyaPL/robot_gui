#include "robot_gui/robot_gui_node.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

RobotGuiNode::RobotGuiNode(ros::NodeHandle &nh)
    :n(nh),
    robot_info_text("(no /robot_info received yet)"),
    frame(cv::Size(600, 950), CV_8UC3),
    window_name("Robot Info GUI"),
    lin_step(0.05),
    ang_step(0.10),
    target_lin_x(0.0),
    target_ang_z(0.0),
    distance_text("Press Get Distance"),
    have_cmd_vel(false),
    have_odom(false),
    publish_hz(30){
    
    sub_robot_info = n.subscribe("/robot_info", 10, &RobotGuiNode::robotInfoCb, this);
    sub_cmd_vel = n.subscribe("/cooper_1/cmd_vel", 10, &RobotGuiNode::cmdVelCb, this);
    sub_odom = n.subscribe("/cooper_1/odom", 10, &RobotGuiNode::odomCb, this);

    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cooper_1/cmd_vel", 10);

    srv_get_distance = n.serviceClient<std_srvs::Trigger>("/get_distance");
    srv_reset_distance = n.serviceClient<std_srvs::Empty>("/reset_distance");

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

void RobotGuiNode::odomCb(const nav_msgs::Odometry::ConstPtr &msg){
    last_odom = *msg;
    have_odom = true;
}

void RobotGuiNode::run() {
    ros::Rate rate(publish_hz);
    while (ros::ok()){
        ros::spinOnce();

        frame = cv::Scalar(49, 52, 49);

        //header
        cvui::text(frame, 20, 15, "Robot GUI", 0.6, 0xFFFFFF);

        //General Info Area
        cvui::window(frame, 20, 40, 560, 180, "General Info");
        cvui::printf(frame, 35, 85, 0.45, 0xFFFFFF, "%s", robot_info_text.c_str());

        //Teleop Buttons
        const int tele_x = 20;
        const int tele_y = 280;
        const int tele_w = 560;
        const int tele_h = 170;

        cvui::window(frame, tele_x, tele_y, tele_w, tele_h, "Teleoperation");

        //button sizes
        const int bw = 110;
        const int bh = 45;

        // center anchor
        const int cx = tele_x + tele_w / 2;
        const int cy = tele_y + tele_h / 2;

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

        //Current Velocities
        const int vel_y = 500;
        const int vel_w = 275;
        const int vel_h = 90;

        double shown_lin = have_cmd_vel ? last_cmd_vel.linear.x : target_lin_x;
        double shown_ang = have_cmd_vel ? last_cmd_vel.angular.z : target_ang_z;

        cvui::window(frame, 20, vel_y, vel_w, vel_h, "Linear Velocity");
        cvui::printf(frame, 40, vel_y+55, 0.60, 0xFFFFFF, "%.2f m/s", shown_lin);

        cvui::window(frame, 305, vel_y, vel_w, vel_h, "Angular Velocity");
        cvui::printf(frame, 325, vel_y+55, 0.60, 0xFFFFFF, "%.2f rad/s", shown_ang);

        //Odometry Position
        const int pos_y = 610;
        const int pos_w = 180;
        const int pos_h = 110;

        double x = 0.0, y = 0.0, z = 0.0;
        if(have_odom){
            x = last_odom.pose.pose.position.x;
            y = last_odom.pose.pose.position.y;
            z = last_odom.pose.pose.position.z;
        }

        //X
        cvui::window(frame, 20, pos_y, pos_w, pos_h, "X");
        cvui::printf(frame, 40,  pos_y + 70, 0.60, 0xFFFFFF, "%.2f", x);

        //Y
        cvui::window(frame, 210, pos_y, pos_w, pos_h, "Y");
        cvui::printf(frame, 230, pos_y + 70, 0.60, 0xFFFFFF, "%.2f", y);

        //Z
        cvui::window(frame, 400, pos_y, pos_w, pos_h, "Z");
        cvui::printf(frame, 420, pos_y + 70, 0.60, 0xFFFFFF, "%.2f", z);

        if (!have_odom) {
            cvui::text(frame, 20, pos_y + pos_h + 10, "Waiting for /cooper_1/odom ...", 0.45, 0xB0B0B0);
        }

        // Distance Travelled Call
        const int dist_y = 740;
        cvui::window(frame, 20, dist_y, 560, 140, "Distance Travelled Call");
        if (cvui::button(frame, 40, dist_y+50, 160, 45, "Get Distance")) {
            std_srvs::Trigger srv;
            if (srv_get_distance.call(srv)) {
                distance_text = srv.response.message;
            }
            else{
                distance_text = "Failed calling /get_distance";
            }
        }

        if (cvui::button(frame, 220, dist_y+50, 160, 45, "Reset Distance")) {
            std_srvs::Empty srv;
            if (srv_reset_distance.call(srv)) {
                distance_text = "0.00";
            }
            else{
                distance_text = "Failed called /reset_distance";
            }
        }
        cvui::printf(frame, 40, dist_y + 110, 0.55, 0xFFFFFF, "Distance: %s", distance_text.c_str());

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