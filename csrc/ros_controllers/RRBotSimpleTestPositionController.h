//
// Created by dstephen on 9/24/15.
//

#ifndef PROJECT_RRBOTSINEWAVEPOSITIONCONTROLLER_H
#define PROJECT_RRBOTSINEWAVEPOSITIONCONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
//#include <

namespace ihmc_ros_control
{
    class RRBotSimpleTestPositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {

    public:
        RRBotSimpleTestPositionController();

        virtual ~RRBotSimpleTestPositionController();

        virtual void update(const ros::Time &time, const ros::Duration &period);

        virtual bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &controller_nh);

        virtual bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &root_nh,
                          ros::NodeHandle &controller_nh);

    private:
        std::vector<hardware_interface::JointHandle> joints;
        double joint2PositionCommand;
        double directionFactor;
    };

}

#endif //PROJECT_RRBOTSINEWAVEPOSITIONCONTROLLER_H
