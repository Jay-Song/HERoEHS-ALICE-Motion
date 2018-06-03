/*
 * alice_manager_python_node.cpp
 *
 *  Created on: May 16, 2018
 *      Author: robotemperor
 */

#include "robotis_controller/robotis_controller.h"
#include "alice_base_module/base_module.h"
#include "alice_leg_module/alice_leg_module.h"
#include "alice_upper_body_module/alice_upper_body_module.h"

using namespace alice;
using namespace alice_leg_module;
using namespace alice_upper_body_module;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "alice_Manager_Python");
    ros::NodeHandle nh;

    ROS_INFO("manager->init");
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

    /* Load ROS Parameter */
    std::string offset_file = nh.param<std::string>("offset_file_path", "");
    std::string robot_file  = nh.param<std::string>("robot_file_path", "");

    std::string init_file   = nh.param<std::string>("init_file_path", "");

    /* gazebo simulation */
    controller->gazebo_mode_ = nh.param<bool>("gazebo", false);
    if(controller->gazebo_mode_ == true)
    {
        ROS_WARN("SET TO GAZEBO MODE!");
        std::string robot_name = nh.param<std::string>("gazebo_robot_name", "");
        if(robot_name != "")
            controller->gazebo_robot_name_ = robot_name;
    }

    if(robot_file == "")
    {
        ROS_ERROR("NO robot file path in the ROS parameters.");
        return -1;
    }

    if(controller->initialize(robot_file, init_file) == false)
    {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }

    if(offset_file != "")
        controller->loadOffset(offset_file);

    sleep(1);
    controller->addMotionModule((robotis_framework::MotionModule*)BaseModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)AliceLegModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)UpperBodyModule::getInstance());

    controller->startTimer();

    while(ros::ok())
    {
      usleep(1000*1000);
    }

    return 0;
}



