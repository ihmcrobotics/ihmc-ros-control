#include "IHMCWholeRobotControlJavaBridge.h"

#include "NativeIMUHandleHolder.h"
#include "NativeForceTorqueSensorHandleHolder.h"

#include <pluginlib/class_list_macros.h>

JNIEXPORT jboolean JNICALL addJointStateToBufferDelegate
        (JNIEnv *env, jobject obj, jlong thisPtr, jstring str)
{
    const char * cstr = env->GetStringUTFChars(str, 0);
    if(cstr != NULL)
    {
        jboolean result = ((ihmc_ros_control::IHMCWholeRobotControlJavaBridge*) thisPtr)->addJointStateToBuffer(std::string(cstr));
        env->ReleaseStringUTFChars(str, cstr);
        return result;
    }
    return false;
}

JNIEXPORT jboolean JNICALL addPositionJointToBufferDelegate
        (JNIEnv *env, jobject obj, jlong thisPtr, jstring str)
{
    const char * cstr = env->GetStringUTFChars(str, 0);
    if(cstr != NULL)
    {
        jboolean result = ((ihmc_ros_control::IHMCWholeRobotControlJavaBridge *) thisPtr)->addPositionJointToBuffer(std::string(cstr));
        env->ReleaseStringUTFChars(str, cstr);

        return result;
    }
    else
    {
        return false;
    }
}

JNIEXPORT jboolean JNICALL addIMUToBufferDelegate
  (JNIEnv *env, jobject obj, jlong thisPtr, jstring str)
{
    const char * cstr = env->GetStringUTFChars(str, 0);
    if(cstr != NULL)
    {
        jboolean result = ((ihmc_ros_control::IHMCWholeRobotControlJavaBridge *) thisPtr)->addIMUToBuffer(std::string(cstr));
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
        jboolean result = ((ihmc_ros_control::IHMCWholeRobotControlJavaBridge *) thisPtr)->addForceTorqueSensorToBuffer(std::string(cstr));
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
    IHMCWholeRobotControlJavaBridge::IHMCWholeRobotControlJavaBridge() :
        ihmcRosControlJavaBridge()
    {
        state_ = ControllerState::CONSTRUCTED;
    }

    IHMCWholeRobotControlJavaBridge::~IHMCWholeRobotControlJavaBridge()
    {
    }

    bool IHMCWholeRobotControlJavaBridge::initRequest(hardware_interface::RobotHW* robot_hw,
                                                      ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
                                                      controller_interface::ControllerBase::ClaimedResources& claimed_resources)
    {

        // check if construction finished cleanly
        if (state_ != ControllerState::CONSTRUCTED){
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

            if(!ihmcRosControlJavaBridge.isAssignableFrom(mainClass, wholeRobotControlInterfaceClass))
            {
                ROS_ERROR_STREAM(mainClass << " does not extend " << wholeRobotControlInterfaceClass);
                return false;
            }

            if(!ihmcRosControlJavaBridge.registerNativeMethod(wholeRobotControlInterfaceClass, "addPositionJointToBufferN", "(JLjava/lang/String;)Z", (void*)&addPositionJointToBufferDelegate))
            {
                ROS_ERROR("Cannot register addPositionJointToBufferN");
                return false;
            }

            if(!ihmcRosControlJavaBridge.registerNativeMethod(wholeRobotControlInterfaceClass, "addJointStateToBufferN", "(JLjava/lang/String;)Z", (void*)&addJointStateToBufferDelegate))
            {
                ROS_ERROR("Cannot register addJointStateToBufferN");
                return false;
            }

            if(!ihmcRosControlJavaBridge.registerNativeMethod(wholeRobotControlInterfaceClass, "addIMUToBufferN", "(JLjava/lang/String;)Z", (void*)&addIMUToBufferDelegate))
            {
                ROS_ERROR("Cannot register addIMUToBufferN");
                return false;
            }

            if(!ihmcRosControlJavaBridge.registerNativeMethod(wholeRobotControlInterfaceClass, "addForceTorqueSensorToBufferN", "(JLjava/lang/String;)Z", (void*)&addForceTorqueSensorToBufferDelegate))
            {
                ROS_ERROR("Cannot register addForceTorqueSensorToBufferN");
                return false;
            }

            imuSensorInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();
            forceTorqueSensorInterface = robot_hw->get<hardware_interface::ForceTorqueSensorInterface>();
            positionJointInterface = robot_hw->get<hardware_interface::PositionJointInterface>();
            jointStateInterface = robot_hw->get<hardware_interface::JointStateInterface>();

            if(ihmcRosControlJavaBridge.createController(mainClass, (long long) this))
            {
                hardware_interface::InterfaceResources iface_res(getHardwareInterfaceType(), hw->getClaims());
                claimed_resources.assign(1, iface_res);
                hw->clearClaims();

                // success
                state_ = ControllerState::INITIALIZED;
                return true;

            }
        }

        return false;
    }

    void IHMCWholeRobotControlJavaBridge::starting(const ros::Time &time)
    {
        ihmcRosControlJavaBridge.starting(time);
    }

    void IHMCWholeRobotControlJavaBridge::update(const ros::Time &time, const ros::Duration &period)
    {
        ihmcRosControlJavaBridge.update(time, period);
    }

    void IHMCWholeRobotControlJavaBridge::stopping(const ros::Time &time)
    {
        ihmcRosControlJavaBridge.stopping(time);
    }

    bool IHMCWholeRobotControlJavaBridge::addJointStateToBuffer(std::string jointName) {
        try
        {
            const hardware_interface::JointStateHandle& handle = jointStateInterface->getHandle(jointName);
            NativeJointStateHandleHolder* holder = new NativeJointStateHandleHolder(handle);
            ihmcRosControlJavaBridge.addUpdatable(holder);
            return true;
        }
        catch(hardware_interface::HardwareInterfaceException e)
        {
            ROS_ERROR_STREAM(e.what());
            return false;
        }
    }

    bool IHMCWholeRobotControlJavaBridge::addPositionJointToBuffer(std::string jointName)
    {
        try
        {
            const hardware_interface::JointHandle& handle = positionJointInterface->getHandle(jointName);
            NativeJointHandleHolder* holder = new NativeJointHandleHolder(handle);
            ihmcRosControlJavaBridge.addUpdatable(holder);
            return true;
        }
        catch(hardware_interface::HardwareInterfaceException e)
        {
            ROS_ERROR_STREAM(e.what());
            return false;
        }
    }

    bool IHMCWholeRobotControlJavaBridge::addIMUToBuffer(std::string imuName)
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

    bool IHMCWholeRobotControlJavaBridge::addForceTorqueSensorToBuffer(std::string forceTorqueSensorName)
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
        ihmc_ros_control::IHMCWholeRobotControlJavaBridge,
        controller_interface::ControllerBase)
