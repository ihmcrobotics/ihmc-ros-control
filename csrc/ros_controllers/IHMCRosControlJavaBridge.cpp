#include "IHMCRosControlJavaBridge.h"
#include "NativeJointHandleHolder.h"

#include <iostream>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <jni.h>
#include <hardware_interface/joint_command_interface.h>


JNIEXPORT jboolean JNICALL addJointToBufferDelegate
  (JNIEnv *env, jobject obj, jlong thisPtr, jstring str)
{
    const char * cstr = env->GetStringUTFChars(str, 0);
    jboolean result = ((ihmc_ros_control::IHMCRosControlJavaBridge*) thisPtr)->addJointToBuffer(std::string(cstr));
    env->ReleaseStringUTFChars(str, cstr);

    return result;
}


JNIEXPORT jobject JNICALL createReadBufferDelegate
  (JNIEnv *env, jobject obj, jlong thisPtr)
{
    return ((ihmc_ros_control::IHMCRosControlJavaBridge*) thisPtr)->createReadBuffer(env);
}

JNIEXPORT jobject JNICALL createWriteBufferDelegate
  (JNIEnv *env, jobject obj, jlong thisPtr)
{
    return ((ihmc_ros_control::IHMCRosControlJavaBridge*) thisPtr)->createWriteBuffer(env);
}


namespace ihmc_ros_control
{
    IHMCRosControlJavaBridge::IHMCRosControlJavaBridge() :
            launcher(nullptr), controllerObject(nullptr), updateMethod(nullptr), hardwareInterface(nullptr), stateBuffer(nullptr), commandBuffer(nullptr)
    {

    }

    IHMCRosControlJavaBridge::~IHMCRosControlJavaBridge()
    {
        if(launcher)
        {
            ROS_INFO("Stopping VM");
            launcher->attachCurrentThread();
            if(controllerObject)
            {
                launcher->release(controllerObject);
            }

            if(updateMethod)
            {
                launcher->release(updateMethod);
            }

            launcher->detachCurrentThread();

            delete launcher;
        }

        if(stateBuffer)
        {
            delete stateBuffer;
        }

        if(commandBuffer)
        {
            delete commandBuffer;
        }


        for(std::vector<NativeUpdateableInterface*>::iterator it = updateables.begin() ; it != updateables.end(); ++it)
        {
            delete (*it);
        }
    }


    void IHMCRosControlJavaBridge::update(const ros::Time &time, const ros::Duration &period)
    {

        int stateIndex = 0;
        for(std::vector<NativeUpdateableInterface*>::iterator it = updateables.begin() ; it != updateables.end(); ++it)
        {
            (*it)->readStateIntoBuffer(stateIndex, stateBuffer);
        }
        launcher->call(updateMethod, controllerObject, time.toNSec(), period.toNSec());

        int commandIndex = 0;
        for(std::vector<NativeUpdateableInterface*>::iterator it = updateables.begin() ; it != updateables.end(); ++it)
        {
            (*it)->writeCommandIntoBuffer(commandIndex, commandBuffer);
        }
    }

    bool IHMCRosControlJavaBridge::startJVM(hardware_interface::EffortJointInterface *hw, std::string jvmArguments, std::string mainClass, std::string workingDirectory)
    {
        ROS_INFO_STREAM("Starting JVM with arguments: " << jvmArguments);
        launcher = new Launcher(jvmArguments);
        if(!launcher->startVM(workingDirectory))
        {
            ROS_ERROR("Cannot start Java VM. If you previously ran a Java controller, limitations in the Java JNI Invocation API prohibit restarting the JVM within a single process. ");
            return false;
        }


        updateMethod = launcher->getJavaMethod(rosControlInterfaceClass, "updateFromNative", "(JJ)V");
        if(!updateMethod)
        {
            ROS_ERROR("Cannot find update method");
            return false;
        }

        if(!launcher->registerNativeMethod(rosControlInterfaceClass, "addJointToBufferN", "(JLjava/lang/String;)Z", (void*)&addJointToBufferDelegate))
        {
            ROS_ERROR("Cannot register addJointToBufferN");
            return false;
        }
        if(!launcher->registerNativeMethod(rosControlInterfaceClass, "createReadBuffer", "(J)Ljava/nio/ByteBuffer;", (void*)&createReadBufferDelegate))
        {
            ROS_ERROR("Cannot register createReadBuffer");
            return false;
        }
        if(!launcher->registerNativeMethod(rosControlInterfaceClass, "createWriteBuffer", "(J)Ljava/nio/ByteBuffer;", (void*)&createWriteBufferDelegate))
        {
            ROS_ERROR("Cannot register createWriteBuffer");
            return false;
        }

        hardwareInterface = hw;

        return true;
    }

    bool IHMCRosControlJavaBridge::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &controller_nh)
    {
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

        if(startJVM(hw, jvmArguments, mainClass, workingDirectory))
        {
            if(!launcher->isAssignableFrom(mainClass, rosControlInterfaceClass))
            {
                ROS_ERROR_STREAM(mainClass << " does not extend " << rosControlInterfaceClass);
                return false;
            }
            return createController(mainClass);
        }
        else
        {
            return false;
        }
    }

    bool IHMCRosControlJavaBridge::createController(std::string mainClass)
    {
        JavaMethod* constructor = launcher->getJavaMethod(mainClass, "<init>", "()V");
        if(!constructor)
        {
            ROS_ERROR_STREAM("Cannot find a no-argument constructor for " << mainClass);
            return false;
        }

        JavaMethod* initMethod = launcher->getJavaMethod(rosControlInterfaceClass, "initFromNative", "(J)V");
        if(!initMethod)
        {
            ROS_ERROR("Cannot find init method");
            return false;
        }
        controllerObject = launcher->createObject(constructor);
        if(!controllerObject)
        {
            ROS_ERROR("Cannot create controller object");
            return false;
        }
        launcher->call(initMethod, controllerObject, (long long) this);

        launcher->release(constructor);
        launcher->release(initMethod);

        launcher->detachCurrentThread();

        return true;
    }

    void IHMCRosControlJavaBridge::starting(const ros::Time &time)
    {
        launcher->attachCurrentThread();
    }
    void IHMCRosControlJavaBridge::stopping(const ros::Time &time)
    {
        launcher->detachCurrentThread();
    }

    bool IHMCRosControlJavaBridge::registerNativeMethod(std::string className, std::string method, std::string signature, void *functionPointer)
    {
        if(launcher)
        {
            return launcher->registerNativeMethod(className, method, signature, functionPointer);
        }
        else
        {
            return false;
        }
    }

    bool IHMCRosControlJavaBridge::isAssignableFrom(std::string subclass, std::string superclass)
    {
        if(launcher)
        {
            return launcher->isAssignableFrom(subclass, superclass);
        }
        else
        {
            return false;
        }
    }

    jobject IHMCRosControlJavaBridge::createReadBuffer(JNIEnv *env)
    {


        int readSize = 0;
        for(std::vector<NativeUpdateableInterface*>::iterator it = updateables.begin() ; it != updateables.end(); ++it)
        {
            readSize += (*it)->stateSize();
        }

        stateBuffer = new double[readSize];
        return env->NewDirectByteBuffer(stateBuffer, sizeof(double) * readSize);
    }

    jobject IHMCRosControlJavaBridge::createWriteBuffer(JNIEnv *env)
    {
        int writeSize = 0;
        for(std::vector<NativeUpdateableInterface*>::iterator it = updateables.begin() ; it != updateables.end(); ++it)
        {
            writeSize += (*it)->commandSize();
        }

        commandBuffer = new double[writeSize];
        return env->NewDirectByteBuffer(commandBuffer, sizeof(double) * writeSize);
    }

    bool IHMCRosControlJavaBridge::addJointToBuffer(std::string jointName)
    {
        try
        {
            const hardware_interface::JointHandle& handle = hardwareInterface->getHandle(jointName);
            NativeJointHandleHolder* holder = new NativeJointHandleHolder(handle);
            updateables.push_back(holder);
            return true;
        }
        catch(hardware_interface::HardwareInterfaceException e)
        {
            ROS_ERROR_STREAM(e.what());
            return false;
        }
    }

    void IHMCRosControlJavaBridge::addUpdatable(NativeUpdateableInterface* updateable)
    {
        updateables.push_back(updateable);
    }
}

PLUGINLIB_EXPORT_CLASS(
        ihmc_ros_control::IHMCRosControlJavaBridge,
        controller_interface::ControllerBase)
