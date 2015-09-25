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

        SimplePDController* joint1Controller = this->jointControllersMap.at(std::string("joint1"));
        joint1Controller->updateJointHandle(1.0, 0.0);
        
        SimplePDController* joint2Controller = this->jointControllersMap.at(std::string("joint2"));
        joint2Controller->updateJointHandle(this->joint2PositionCommand, 0.0);

        this->joint2PositionCommand += (0.001 * this->directionFactor);

        if (this->joint2PositionCommand > 1.0) {
            this->directionFactor = -1.0;
        }
        else if (this->joint2PositionCommand < -1.0) {
            this->directionFactor = 1.0;
        }
    }

    bool RRBotSimpleTestPositionController::init(hardware_interface::EffortJointInterface *hw,
                                                 ros::NodeHandle &controller_nh) {

        const hardware_interface::JointHandle &joint1Handle = hw->getHandle("joint1");
        auto joint1Controller = new SimplePDController(joint1Handle, true, 10000.0, 10.0);
        const hardware_interface::JointHandle &joint2Handle = hw->getHandle("joint2");
        auto joint2Controller = new SimplePDController(joint2Handle, true, 100.0, 10.0);

        this->jointControllersMap.insert(std::pair<std::string, SimplePDController*>(std::string("joint1"), joint1Controller));
        this->jointControllersMap.insert(std::pair<std::string, SimplePDController*>(std::string("joint2"), joint2Controller));

        return true;
    }
}

PLUGINLIB_EXPORT_CLASS(
        ihmc_ros_control::RRBotSimpleTestPositionController,
        controller_interface::ControllerBase)
