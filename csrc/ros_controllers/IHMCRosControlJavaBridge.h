#ifndef IHMCROSCONTROLJAVABRIDGE_H
#define IHMCROSCONTROLJAVABRIDGE_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_position_controller.h>

#include <jvmLauncher/launcher.h>

namespace ihmc_ros_control
{
    class IHMCRosControlJavaBridge : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        IHMCRosControlJavaBridge();

        ~IHMCRosControlJavaBridge();

        void update(const ros::Time &time, const ros::Duration &period);

        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &controller_nh);

        void starting(const ros::Time &time) override;

    private:
        Launcher* launcher;
    };
}
#endif // IHMCROSCONTROLJAVABRIDGE_H
