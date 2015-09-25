//
// Created by dstephen on 9/24/15.
//

#ifndef PROJECT_RRBOTSINEWAVEPOSITIONCONTROLLER_H
#define PROJECT_RRBOTSINEWAVEPOSITIONCONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_position_controller.h>

namespace ihmc_ros_control
{
    class RRBotSimpleTestPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

    public:
        RRBotSimpleTestPositionController();

        virtual ~RRBotSimpleTestPositionController();

        virtual void update(const ros::Time &time, const ros::Duration &period);

        virtual bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &controller_nh);

        virtual void starting(const ros::Time &time) override;

    private:
        double joint2PositionCommand;
        double directionFactor;
        effort_controllers::JointPositionController joint1Controller;
        effort_controllers::JointPositionController joint2Controller;
    };

}

#endif //PROJECT_RRBOTSINEWAVEPOSITIONCONTROLLER_H
