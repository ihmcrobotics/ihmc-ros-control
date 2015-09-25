//
// Created by dstephen on 9/24/15.
//

#include "RRBotSimpleTestPositionController.h"
#include <pluginlib/class_list_macros.h>

namespace ihmc_ros_control {

    RRBotSimpleTestPositionController::RRBotSimpleTestPositionController() {
        this->joint2PositionCommand = 0.0;
        this->directionFactor = 1.0;
    }

    RRBotSimpleTestPositionController::~RRBotSimpleTestPositionController() {

    }

    void RRBotSimpleTestPositionController::update(const ros::Time &time, const ros::Duration &period) {
        for (auto joint : this->joints) {
            if (joint.getName() == "joint1") {
                joint.setCommand(1.0);
            }

            if (joint.getName() == "joint2") {
                joint.setCommand(this->joint2PositionCommand);
            }
        }

        this->joint2PositionCommand += (0.1 * this->directionFactor);

        if (this->joint2PositionCommand > 1.0) {
            this->directionFactor = -1.0;
        }
        else if (this->joint2PositionCommand < -1.0) {
            this->directionFactor = 1.0;
        }
    }

    bool RRBotSimpleTestPositionController::init(hardware_interface::PositionJointInterface *hw,
                                                 ros::NodeHandle &controller_nh) {

        ROS_INFO("This is a test.");
        ROS_INFO("Size of resources: %lu", hw->getNames().size());

        for(auto jointName : hw->getNames())
        {
            ROS_INFO("%s", jointName.c_str());
        }

        bool initSuccess = controller_interface::Controller<hardware_interface::PositionJointInterface>::init(hw,
                                                                                                              controller_nh);

        if (initSuccess) {
            this->joints.push_back(hw->getHandle("joint1"));
            this->joints.push_back(hw->getHandle("joint2"));
        }
        return initSuccess;
    }

    bool RRBotSimpleTestPositionController::init(hardware_interface::PositionJointInterface *hw,
                                                 ros::NodeHandle &root_nh,
                                                 ros::NodeHandle &controller_nh) {
        ROS_INFO("This is a test.");
        ROS_INFO("Size of resources: %lu", hw->getNames().size());

        for(auto jointName : hw->getNames())
        {
            ROS_INFO("%s", jointName.c_str());
        }

        bool initSuccess = controller_interface::Controller<hardware_interface::PositionJointInterface>::init(hw,
                                                                                                              root_nh,
                                                                                                              controller_nh);
        if (initSuccess) {
            this->joints.push_back(hw->getHandle("joint1"));
            this->joints.push_back(hw->getHandle("joint2"));
        }
        return initSuccess;
    }
}

PLUGINLIB_EXPORT_CLASS(
        ihmc_ros_control::RRBotSimpleTestPositionController,
        controller_interface::ControllerBase)
