//
// Created by dstephen on 9/24/15.
//

#ifndef PROJECT_RRBOTSINEWAVEPOSITIONCONTROLLER_H
#define PROJECT_RRBOTSINEWAVEPOSITIONCONTROLLER_H

#include <unordered_map>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_position_controller.h>

#include "SimplePDController.h"

namespace ihmc_ros_control
{
    class RRBotSimpleTestPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

    public:
        RRBotSimpleTestPositionController();

        virtual ~RRBotSimpleTestPositionController();

        virtual void update(const ros::Time &time, const ros::Duration &period);

        virtual bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &controller_nh);

    private:
        double joint2PositionCommand;
        double directionFactor;
        std::unordered_map<std::string, SimplePDController*> jointControllersMap;
    };

}

#endif //PROJECT_RRBOTSINEWAVEPOSITIONCONTROLLER_H
