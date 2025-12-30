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
    frame(cv::Size(420, 960), CV_8UC3),
    window_name("Robot Info GUI"),
    lin_step(0.05),
    ang_step(0.10),
    target_lin_x(0.0),
    target_ang_z(0.0),
    distance_text("0.00"),
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

        // Layout constants
        const int FRAME_W = frame.cols;
        const int FRAME_H = frame.rows;

        const int M   = 20;   // margin
        const int GAP = 10;
        const int W   = FRAME_W - 2 * M;

        const int INFO_Y = 40;
        const int INFO_H = 170;

        const int TELE_Y = INFO_Y + INFO_H + GAP;
        const int TELE_H = 260;

        const int VEL_Y  = TELE_Y + TELE_H + GAP;
        const int VEL_H  = 80;

        const int POS_Y  = VEL_Y + VEL_H + GAP;
        const int POS_H  = 170;

        const int DIST_Y = POS_Y + POS_H + GAP;
        const int DIST_H = 140;

        //Header
        cvui::text(frame, M, 15, "Robot GUI", 0.6, 0xFFFFFF);

        //General Info
        cvui::window(frame, M, INFO_Y, W, INFO_H, "Info");
        cvui::printf(frame, M + 15, INFO_Y + 45, 0.45, 0xFFFFFF, "%s", robot_info_text.c_str());

        //Teleop
        cvui::window(frame, M, TELE_Y, W, TELE_H, "Teleoperation");

        const int bw = 90;
        const int bh = 45;
        const int gap_btn = 12;

        //canter anchor
        const int cx = M + W/2;
        const int cy = TELE_Y + 125;

        //Forward
        if (cvui::button(frame, cx - bw/2, cy - (bh + gap_btn), bw, bh, "Forward")){
            target_lin_x += lin_step;
        }

        //Left
        if (cvui::button(frame, cx - bw/2 - (bw + gap_btn), cy, bw, bh, "Left")) {
            target_ang_z += ang_step;
        }

        //Stop
        if (cvui::button(frame, cx - bw/2, cy, bw, bh, "Stop")) {
            target_lin_x = 0.0; target_ang_z = 0.0;
        }

        //Right
        if (cvui::button(frame, cx - bw/2 + bw + gap_btn, cy, bw, bh, "Right")){
            target_ang_z -= ang_step;
        }

        //Backward
        if (cvui::button(frame, cx - bw/2, cy + (bh + gap_btn), bw, bh, "Backward")) {
            target_lin_x -= lin_step;
        }

        //Current Velocities
        double shown_lin = have_cmd_vel ? last_cmd_vel.linear.x  : target_lin_x;
        double shown_ang = have_cmd_vel ? last_cmd_vel.angular.z : target_ang_z;

        const int vel_w = (W - GAP) / 2;

        cvui::window(frame, M, VEL_Y, vel_w, VEL_H, "Linear velocity:");
        cvui::printf(frame, M + 15, VEL_Y + 55, 0.65, 0xFFFFFF, "%.2f m/sec", shown_lin);

        cvui::window(frame, M + vel_w + GAP, VEL_Y, vel_w, VEL_H, "Angular velocity:");
        cvui::printf(frame, M + vel_w + GAP + 15, VEL_Y + 55, 0.65, 0xFFFFFF, "%.2f rad/sec", shown_ang);

        //Robot Position
        cvui::window(frame, M, POS_Y, W, POS_H, "Robot position based off odometry");

        double px = 0.0, py = 0.0, pz = 0.0;
        if (have_odom) {
        px = last_odom.pose.pose.position.x;
        py = last_odom.pose.pose.position.y;
        pz = last_odom.pose.pose.position.z;
        }

        const int inner_m = 10;
        const int box_y   = POS_Y + 45;
        const int box_h   = POS_H - 55;
        const int box_w   = (W - 2*inner_m - 2*GAP) / 3;

        const int x1 = M + inner_m;
        const int x2 = x1 + box_w + GAP;
        const int x3 = x2 + box_w + GAP;

        cvui::window(frame, x1, box_y, box_w, box_h, "X");
        cvui::printf(frame, x1 + 18, box_y + 75, 0.75, 0xFFFFFF, "%.2f", px);

        cvui::window(frame, x2, box_y, box_w, box_h, "Y");
        cvui::printf(frame, x2 + 18, box_y + 75, 0.75, 0xFFFFFF, "%.2f", py);

        cvui::window(frame, x3, box_y, box_w, box_h, "Z");
        cvui::printf(frame, x3 + 18, box_y + 75, 0.75, 0xFFFFFF, "%.2f", pz);

        //Distance Travelled
        cvui::window(frame, M, DIST_Y, W, DIST_H, "Distance Travelled");

        const int left_w  = 120;
        const int right_w = W - left_w - GAP;

        //Call/Reset
        cvui::window(frame, M, DIST_Y + 35, left_w, DIST_H - 45, " ");

        if (cvui::button(frame, M + 5, DIST_Y + 55, left_w - 10, 35, "Call")) {
            std_srvs::Trigger srv;
            if (srv_get_distance.call(srv)){
                distance_text = srv.response.message;
            }
            else{
                distance_text = "ERR";
            }
        }

        if (cvui::button(frame, M + 5, DIST_Y + 95, left_w - 10, 35, "Reset")) {
            std_srvs::Empty srv;
            if (srv_reset_distance.call(srv)){
                distance_text = "0.00";
            }
            else{
                distance_text = "ERR";
            }
        }

        // Right panel
        cvui::window(frame, M + left_w + GAP, DIST_Y + 35, right_w, DIST_H - 45, "Distance:");
        cvui::printf(frame, M + left_w + GAP + 18, DIST_Y + 110, 1.00, 0xFFFFFF, "%s", distance_text.c_str());

        //punlish geometry msg
        geometry_msgs::Twist cmd;
        cmd.linear.x  = target_lin_x;
        cmd.angular.z = target_ang_z;
        pub_cmd_vel.publish(cmd);


        //Render
        cvui::update();
        cv::imshow(window_name, frame);
        if (cv::waitKey(1) == 27) break;

        rate.sleep();
    }
}