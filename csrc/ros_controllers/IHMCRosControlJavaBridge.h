#ifndef IHMCROSCONTROLJAVABRIDGE_H
#define IHMCROSCONTROLJAVABRIDGE_H

#include "NativeUpdateableInterface.h"

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_position_controller.h>
#include "../jvmLauncher/launcher.h"
#include <vector>

namespace ihmc_ros_control
{
    const std::string rosControlInterfaceClass = "us.ihmc.rosControl.IHMCRosControlJavaBridge";
    class IHMCRosControlJavaBridge : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        IHMCRosControlJavaBridge();

        virtual ~IHMCRosControlJavaBridge();

        /**
         * @brief update Called cyclically by the realtime thread
         * @param time
         * @param period
         */
        virtual void update(const ros::Time &time, const ros::Duration &period);


        /**
         * @brief init Called by a non-realtime thread to initialize the controller
         * @param hw
         * @param controller_nh
         * @return
         */
        virtual bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &controller_nh);

        /**
         * @brief startJVM Starts the virtual machine, but does not create the controller object
         * @param hw
         * @param jvmArguments
         * @param mainClass
         * @param workingDirectory
         * @return
         */
        bool startJVM(hardware_interface::EffortJointInterface *hw, std::string jvmArguments, std::string mainClass, std::string workingDirectory);

        /**
         * @brief createController creates the controller object
         * @return
         */
        bool createController(std::string mainClass);

        /**
         * @brief starting Called from the Realtime Thread before the first update
         * @param time
         */
        virtual void starting(const ros::Time &time) override;

        /**
         * @brief stopping Called from the Realtime thread after the last update
         */
        virtual void stopping(const ros::Time&);


        /**
         * @brief createReadBuffer Create a buffer to read state from, called from Java in the init() method
         * @param env
         * @return ByteBuffer backed by the stateBuffer
         */
        jobject createReadBuffer(JNIEnv *env);

        /**
         * @brief createWriteBuffer Create a buffer to write command data, called from Java in the init() method
         * @param env
         * @return ByteBuffer backed by the commandBuffer
         */
        jobject createWriteBuffer(JNIEnv *env);

        /**
         * @brief addJointToBuffer Called from Java to create a joint.
         * @param jointName
         */
        bool addJointToBuffer(std::string jointName);

        /**
         * @brief registerNativeMethod Delegate funtion for Launcher::registerNativeMethod()
         * @param className
         * @param method
         * @param signature
         * @param functionPointer
         * @return
         */
        bool registerNativeMethod(std::string className, std::string method, std::string signature, void* functionPointer);

        /**
         * @brief isAssignableFrom Delegate function for Launcher::isAssignableFrom()
         * @param subClass
         * @param superClass
         * @return
         */
        bool isAssignableFrom(std::string subclass, std::string superclass);

        /**
         * @brief addUpdatable Add interface to be updated
         * @param updatable
         */
        void addUpdatable(NativeUpdateableInterface* updatable);
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
