#ifndef IHMCROSCONTROLJAVABRIDGE_H
#define IHMCROSCONTROLJAVABRIDGE_H

#include "NativeUpdateableInterface.h"

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_position_controller.h>
#include <jvmLauncher/launcher.h>
#include <vector>

namespace ihmc_ros_control
{
    const std::string rosControlInterfaceClass = "us.ihmc.rosControl.launcher.IHMCRosControlJavaBridge";
    class IHMCRosControlJavaBridge : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        IHMCRosControlJavaBridge();

        virtual ~IHMCRosControlJavaBridge();

        void update(const ros::Time &time, const ros::Duration &period);

        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &controller_nh);

        void starting(const ros::Time &time) override;

        jobject createReadBuffer(JNIEnv *env);
        jobject createWriteBuffer(JNIEnv *env);

        void addJointToBuffer(std::string jointName);

    private:
        Launcher* launcher;
        jobject controllerObject;
        JavaMethod* updateMethod;

        hardware_interface::EffortJointInterface* hardwareInterface;
        std::vector<NativeUpdateableInterface*> updateables;

        double* stateBuffer;
        double* commandBuffer;
    };
}
#endif // IHMCROSCONTROLJAVABRIDGE_H
