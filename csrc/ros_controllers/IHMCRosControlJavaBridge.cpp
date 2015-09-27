#include "IHMCRosControlJavaBridge.h"
#include "NativeJointHandleHolder.h"

#include <iostream>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <jni.h>
#include <hardware_interface/joint_command_interface.h>
#include <thread>


JNIEXPORT void JNICALL addJointToBufferDelegate
  (JNIEnv *env, jobject obj, jlong thisPtr, jstring str)
{
    const char * cstr = env->GetStringUTFChars(str, 0);
    ((ihmc_ros_control::IHMCRosControlJavaBridge*) thisPtr)->addJointToBuffer(std::string(cstr));
    env->ReleaseStringUTFChars(str, cstr);
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
            std::cout << "Stopping VM" << std::endl;
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

    bool IHMCRosControlJavaBridge::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &controller_nh)
    {
        std::string jvmArguments;
        std::string mainClass;
        std::string workingDirectory;

        if(!controller_nh.getParam("jvm_args", jvmArguments))
        {
            std::cerr << "No jvm_args provided." << std::endl;
            return false;

        }

        if(!controller_nh.getParam("main_class", mainClass))
        {
            std::cerr << "No main_class provided" << std::endl;
            return false;
        }

        if(!controller_nh.getParam("working_dir", workingDirectory))
        {
            std::cout << "No working directory provided. Using current directory" << std::endl;
            workingDirectory = ".";
        }



        std::cout << "Starting JVM with arguments: " << jvmArguments << std::endl;
        launcher = new Launcher(jvmArguments);
        if(!launcher->startVM(workingDirectory))
        {
            std::cerr << "Cannot start Java VM. If you previously ran a Java controller, limitations in the Java JNI Invocation API prohibit restarting the JVM within a single process. " << std::endl;
            return false;
        }

        if(!launcher->isAssignableFrom(mainClass, rosControlInterfaceClass))
        {
            std::cerr << mainClass << " does not extend " << rosControlInterfaceClass << std::endl;
            return false;
        }


        JavaMethod* constructor = launcher->getJavaMethod(mainClass, "<init>", "()V");
        if(!constructor)
        {
            std::cerr << "Cannot find a no-argument constructor for " << mainClass << std::endl;
            return false;
        }

        updateMethod = launcher->getJavaMethod(rosControlInterfaceClass, "updateFromNative", "(JJ)V");
        if(!updateMethod)
        {
            std::cerr << "Cannot find update method" << std::endl;
            return false;
        }

        JavaMethod* initMethod = launcher->getJavaMethod(rosControlInterfaceClass, "initFromNative", "(J)V");
        if(!initMethod)
        {
            std::cerr << "Cannot find init method" << std::endl;
            return false;
        }

        if(!launcher->registerNativeMethod(rosControlInterfaceClass, "addJointToBufferN", "(JLjava/lang/String;)V", (void*)&addJointToBufferDelegate))
        {
            std::cerr << "Cannot register addJointToBufferN" << std::endl;
            return false;
        }
        if(!launcher->registerNativeMethod(rosControlInterfaceClass, "createReadBuffer", "(J)Ljava/nio/ByteBuffer;", (void*)&createReadBufferDelegate))
        {
            std::cerr << "Cannot register createReadBuffer" << std::endl;
            return false;
        }
        if(!launcher->registerNativeMethod(rosControlInterfaceClass, "createWriteBuffer", "(J)Ljava/nio/ByteBuffer;", (void*)&createWriteBufferDelegate))
        {
            std::cerr << "Cannot register createWriteBuffer" << std::endl;
            return false;
        }

        hardwareInterface = hw;

        controllerObject = launcher->createObject(constructor);
        if(!controllerObject)
        {
            std::cerr << "Cannot create controller object" << std::endl;
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

    void IHMCRosControlJavaBridge::addJointToBuffer(std::string jointName)
    {
        const hardware_interface::JointHandle& handle = hardwareInterface->getHandle(jointName);

        NativeJointHandleHolder* holder = new NativeJointHandleHolder(handle);
        updateables.push_back(holder);
    }
}

PLUGINLIB_EXPORT_CLASS(
        ihmc_ros_control::IHMCRosControlJavaBridge,
        controller_interface::ControllerBase)
