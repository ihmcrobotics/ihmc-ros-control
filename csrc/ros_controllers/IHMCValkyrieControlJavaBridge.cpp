#include "IHMCValkyrieControlJavaBridge.h"

#include "NativeIMUHandleHolder.h"
#include "NativeForceTorqueSensorHandleHolder.h"

#include <pluginlib/class_list_macros.h>
#include <jni.h>
#include <hardware_interface/joint_command_interface.h>


JNIEXPORT jboolean JNICALL addIMUToBufferDelegate
  (JNIEnv *env, jobject obj, jlong thisPtr, jstring str)
{
    const char * cstr = env->GetStringUTFChars(str, 0);
    if(cstr != NULL)
    {
        jboolean result = ((ihmc_ros_control::IHMCValkyrieControlJavaBridge*) thisPtr)->addIMUToBuffer(std::string(cstr));
        env->ReleaseStringUTFChars(str, cstr);

        return result;
    }
    else
    {
        return false;
    }
}

JNIEXPORT jboolean JNICALL addForceTorqueSensorToBufferDelegate
  (JNIEnv *env, jobject obj, jlong thisPtr, jstring str)
{
    const char * cstr = env->GetStringUTFChars(str, 0);
    if(cstr != NULL)
    {
        jboolean result = ((ihmc_ros_control::IHMCValkyrieControlJavaBridge*) thisPtr)->addForceTorqueSensorToBuffer(std::string(cstr));
        env->ReleaseStringUTFChars(str, cstr);

        return result;
    }
    else
    {
        return false;
    }
}

namespace ihmc_ros_control
{
    IHMCValkyrieControlJavaBridge::IHMCValkyrieControlJavaBridge() :
        ihmcRosControlJavaBridge()
    {
        state_ = CONSTRUCTED;
    }

    IHMCValkyrieControlJavaBridge::~IHMCValkyrieControlJavaBridge()
    {
    }

    bool IHMCValkyrieControlJavaBridge::initRequest(hardware_interface::RobotHW* robot_hw,
                             ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
                             std::set<std::string> &claimed_resources)
    {

        // check if construction finished cleanly
        if (state_ != CONSTRUCTED){
          ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
          return false;
        }

        hardware_interface::EffortJointInterface* hw = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (!hw)
        {
          ROS_ERROR("Cannot get hardware interface of type hardware_interface::EffortJointInterface");
          return false;
        }
        hw->clearClaims();

        std::string jvmArguments;
        std::string mainClass;
        std::string workingDirectory;

        if(!controller_nh.getParam("jvm_args", jvmArguments))
        {
            ROS_ERROR("No jvm_args provided.");
            return false;

        }

        if(!controller_nh.getParam("main_class", mainClass))
        {
            ROS_ERROR("No main_class provided");
            return false;
        }

        if(!controller_nh.getParam("working_dir", workingDirectory))
        {
            ROS_INFO("No working directory provided. Using current directory");
            workingDirectory = ".";
        }

        if(ihmcRosControlJavaBridge.startJVM(hw, jvmArguments, workingDirectory))
        {

            if(!ihmcRosControlJavaBridge.isAssignableFrom(mainClass, valkyrieControlInterfaceClass))
            {
                ROS_ERROR_STREAM(mainClass << " does not extend " << valkyrieControlInterfaceClass);
                return false;
            }

            if(!ihmcRosControlJavaBridge.registerNativeMethod(valkyrieControlInterfaceClass, "addIMUToBufferN", "(JLjava/lang/String;)Z", (void*)&addIMUToBufferDelegate))
            {
                ROS_ERROR("Cannot register addIMUToBufferN");
                return false;
            }

            if(!ihmcRosControlJavaBridge.registerNativeMethod(valkyrieControlInterfaceClass, "addForceTorqueSensorToBufferN", "(JLjava/lang/String;)Z", (void*)&addForceTorqueSensorToBufferDelegate))
            {
                ROS_ERROR("Cannot register addForceTorqueSensorToBufferN");
                return false;
            }

            imuSensorInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();
            forceTorqueSensorInterface = robot_hw->get<hardware_interface::ForceTorqueSensorInterface>();

            if(ihmcRosControlJavaBridge.createController(mainClass, (long long) this))
            {
                claimed_resources = hw->getClaims();
                hw->clearClaims();

                // success
                state_ = INITIALIZED;
                return true;

            }
        }

        return false;
    }

    void IHMCValkyrieControlJavaBridge::starting(const ros::Time &time)
    {
        ihmcRosControlJavaBridge.starting(time);
    }

    void IHMCValkyrieControlJavaBridge::update(const ros::Time &time, const ros::Duration &period)
    {
        ihmcRosControlJavaBridge.update(time, period);
    }

    void IHMCValkyrieControlJavaBridge::stopping(const ros::Time &time)
    {
        ihmcRosControlJavaBridge.stopping(time);
    }

    bool IHMCValkyrieControlJavaBridge::addIMUToBuffer(std::string imuName)
    {
        try
        {
            const hardware_interface::ImuSensorHandle& handle = imuSensorInterface->getHandle(imuName);
            NativeIMUHandleHolder* holder = new NativeIMUHandleHolder(handle);
            ihmcRosControlJavaBridge.addUpdatable(holder);
            return true;
        }
        catch(hardware_interface::HardwareInterfaceException e)
        {
            ROS_ERROR_STREAM(e.what());
            return false;
        }
    }

    bool IHMCValkyrieControlJavaBridge::addForceTorqueSensorToBuffer(std::string forceTorqueSensorName)
    {
        try
        {
            const hardware_interface::ForceTorqueSensorHandle& handle = forceTorqueSensorInterface->getHandle(forceTorqueSensorName);
            NativeForceTorqueSensorHandleHolder* holder = new NativeForceTorqueSensorHandleHolder(handle);
            ihmcRosControlJavaBridge.addUpdatable(holder);
            return true;
        }
        catch(hardware_interface::HardwareInterfaceException e)
        {
            ROS_ERROR_STREAM(e.what());
            return false;
        }
    }


}

PLUGINLIB_EXPORT_CLASS(
        ihmc_ros_control::IHMCValkyrieControlJavaBridge,
        controller_interface::ControllerBase)
