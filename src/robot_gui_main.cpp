#include <ros/ros.h>
#include "robot_gui/robot_gui_node.h"
#include "ros/node_handle.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "robot_gui_node");
    ros::NodeHandle nh;

    RobotGuiNode gui(nh);
    gui.run();
    return 0;
}