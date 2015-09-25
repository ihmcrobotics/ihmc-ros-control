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

        joint1Controller.setCommand(1.0);

        joint2Controller.setCommand(this->joint2PositionCommand);

        this->joint2PositionCommand += (0.001 * this->directionFactor);

        if (this->joint2PositionCommand > 1.0) {
            this->directionFactor = -1.0;
        }
        else if (this->joint2PositionCommand < -1.0) {
            this->directionFactor = 1.0;
        }

        joint1Controller.update(time, period);
        joint2Controller.update(time, period);
    }

    bool RRBotSimpleTestPositionController::init(hardware_interface::EffortJointInterface *hw,
                                                 ros::NodeHandle &controller_nh) {

        controller_nh.setParam("joint", "joint1");
        controller_nh.setParam("pid/p", 10000.0);
        controller_nh.setParam("pid/i", 0.0);
        controller_nh.setParam("pid/d", 10.0);
        controller_nh.setParam("pid/max", 3.0);
        controller_nh.setParam("pid/min", -3.0);
        joint1Controller.init(hw, controller_nh);

        controller_nh.setParam("joint", "joint2");
        controller_nh.setParam("pid/p", 100.0);
        controller_nh.setParam("pid/i", 0.0);
        controller_nh.setParam("pid/d", 10.0);
        controller_nh.setParam("pid/max", 3.0);
        controller_nh.setParam("pid/min", -3.0);
        joint2Controller.init(hw, controller_nh);

        return true;
    }

    void RRBotSimpleTestPositionController::starting(const ros::Time &time) {
        joint1Controller.starting(time);
        joint2Controller.starting(time);
    }
}

PLUGINLIB_EXPORT_CLASS(
        ihmc_ros_control::RRBotSimpleTestPositionController,
        controller_interface::ControllerBase)
