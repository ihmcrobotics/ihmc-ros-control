#include "launcher.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <algorithm>

#include <jni.h>
#include <vector>
#include <sys/types.h>
#include <unistd.h>
#include <linux/limits.h>
#include <dirent.h>

void Launcher::displayJNIError(std::string prefix, int error)
{
    switch(error)
    {
        case JNI_OK: std::cout << prefix << ": success" << std::endl; return;
        case JNI_ERR: std::cerr << prefix << ": unknown error" << std::endl; return;
        case JNI_EDETACHED: std::cerr << prefix << ": thread detached from the VM" << std::endl; return;
        case JNI_EVERSION : std::cerr << prefix << ": JNI version error" << std::endl; return;
        case JNI_ENOMEM : std::cerr << prefix << ": not enough memory" << std::endl; return;
        case JNI_EEXIST : std::cerr << prefix << ": VM already created" << std::endl; return;
        case JNI_EINVAL : std::cerr << prefix << ": invalid arguments" << std::endl; return;
    }
}

JNIEnv* Launcher::getEnv()
{
    JNIEnv* ret;

    if(!jvm)
    {
        std::cerr << "JVM not started" << std::endl;
        return nullptr;
    }
    jvm->GetEnv((void**) &ret, JNI_VERSION_1_6);
    if(!ret)
    {
        std::cerr << "Cannot get env" << std::endl;
        return nullptr;
    }

    return ret;
}

void Launcher::attachCurrentThread()
{
    if(!jvm)
    {
        std::cerr << "JVM not started" << std::endl;
        return;
    }

    JNIEnv* tmp;
    jvm->AttachCurrentThread((void**)&tmp, 0);
}

void Launcher::detachCurrentThread()
{
    if(!jvm)
    {
        std::cerr << "JVM not started" << std::endl;
        return;
    }

    jvm->DetachCurrentThread();
}

bool Launcher::callBooleanMethod(JavaMethod *method, jobject obj, ...)
{
    JNIEnv* env = getEnv();
    if(!env) return false;

    bool returnValue = false;
    if(env->IsInstanceOf(obj, method->clazz))
    {
        va_list arglist;
        va_start(arglist, obj);
        returnValue = env->CallBooleanMethodV(obj, method->methodID, arglist);
        va_end(arglist);
    }
    else
    {
        std::cerr << __PRETTY_FUNCTION__ << ": Unexpected object type" << std::endl;
    }

    return returnValue;
}

void Launcher::call(JavaMethod *method, jobject obj, ...)
{
    JNIEnv* env = getEnv();
    if(!env) return;

    if(env->IsInstanceOf(obj, method->clazz))
    {
        va_list arglist;
        va_start(arglist, obj);
        env->CallVoidMethodV(obj, method->methodID, arglist);
        va_end(arglist);
    }
    else
    {
        std::cerr << __PRETTY_FUNCTION__ << ": Unexpected object type" << std::endl;
    }
}

void Launcher::call(StaticJavaMethod *method, ...)
{
    JNIEnv* env = getEnv();
    if(!env) return;

    va_list arglist;
    va_start(arglist, method);
    env->CallStaticVoidMethodV(method->clazz, method->methodID, arglist);
    va_end(arglist);
}

Launcher::Launcher(std::string vmOptions)
{
    vmArguments.version = JNI_VERSION_1_6;

    std::istringstream vmOptionsTokens(vmOptions);

    std::vector<std::string> options;
    while(!vmOptionsTokens.eof())
    {
        std::string option;
        std::getline(vmOptionsTokens, option, ' ');
        options.push_back(option);
    }


    JavaVMOption* javaOptions = new JavaVMOption[options.size()];
    for(uint i = 0; i < options.size(); i++)
    {
        javaOptions[i].optionString = new char[options.at(i).length() + 1];
        std::strcpy (javaOptions[i].optionString, options.at(i).c_str());
    }

    vmArguments.nOptions = 1;
    vmArguments.options = javaOptions;
    vmArguments.ignoreUnrecognized = true;
}

bool Launcher::startVM(std::string workingDirectory)
{


    DIR* currentDirectory = opendir(".");
    if(workingDirectory != "." && workingDirectory != "")
    {
        if(chdir(workingDirectory.c_str()) == -1)
        {
            std::cerr << "Cannot change directory to " << workingDirectory << std::endl;
            closedir(currentDirectory);
            return false;
        }

    }

    char temp [PATH_MAX];
    if(getcwd(temp, PATH_MAX) == 0); // Ignore return type
    std::cout << "Starting Java VM from path  " << temp << std::endl;

    JNIEnv* env;
    jint res = JNI_CreateJavaVM(&jvm, (void**) &env, &vmArguments);
    displayJNIError("Started Java VM", res);

    if(workingDirectory != ".")
    {
        if(fchdir(dirfd(currentDirectory)) != 0)
        {
            std::cerr << "Cannot return to previous working directory" << std::endl;
        }
    }
    closedir(currentDirectory);
    if(res == JNI_OK)
    {
        return true;
    }
    else
    {
        jvm = nullptr;
        return false;
    }
}

jclass Launcher::getClass(std::string className)
{
    JNIEnv* env = getEnv();
    if(!env) return nullptr;

    std::string classNameCopy(className);
    std::replace (classNameCopy.begin(), classNameCopy.end(), '.', '/');

    jclass cls = env->FindClass(classNameCopy.c_str());
    if(!cls)
    {
        std::cerr << "Cannot find class " << classNameCopy << std::endl;
        return nullptr;
    }
    return cls;
}

jobject Launcher::createObject(JavaMethod *constructor, ...)
{
    JNIEnv* env = getEnv();
    if(!env) return nullptr;


    va_list arglist;
    va_start(arglist, constructor);
    jobject newObject = env->NewObjectV(constructor->clazz, constructor->methodID, arglist);
    va_end(arglist);

    return env->NewGlobalRef(newObject);

}

void Launcher::release(jobject object)
{
    JNIEnv* env = getEnv();
    if(!env) return;

    env->DeleteGlobalRef(object);
}

StaticJavaMethod* Launcher::getStaticJavaMethod(std::string className, std::string methodName, std::string signature)
{

    JNIEnv* env = getEnv();
    if(!env) return nullptr;

    jclass cls = getClass(className);
    if(!cls) return nullptr;


    jmethodID mid = env->GetStaticMethodID(cls, methodName.c_str(), signature.c_str());
    if(!mid)
    {
        std::cerr << "Cannot find method " << methodName << signature << std::endl;
        return nullptr;
    }

    StaticJavaMethod* method = new StaticJavaMethod();
    method->clazz = (jclass) env->NewGlobalRef(cls);
    method->methodID = mid;

    return method;
}

void Launcher::release(StaticJavaMethod *method)
{
    release(method->clazz);
    delete method;
}

void Launcher::release(JavaMethod *method)
{
    release(method->clazz);
    delete method;
}

JavaMethod* Launcher::getJavaMethod(std::string className, std::string methodName, std::string signature)
{

    JNIEnv* env = getEnv();
    if(!env) return nullptr;

    jclass cls = getClass(className);
    if(!cls) return nullptr;


    jmethodID mid = env->GetMethodID(cls, methodName.c_str(), signature.c_str());
    if(!mid)
    {
        std::cerr << "Cannot find method " << methodName << "()" << std::endl;
        return nullptr;
    }

    JavaMethod* method = new JavaMethod();
    method->clazz = (jclass) env->NewGlobalRef(cls);
    method->methodID = mid;

    return method;
}

bool Launcher::registerNativeMethod(std::string className, std::string methodName, std::string signature, void *functionPointer)
{
    JNIEnv* env = getEnv();
    if(!env) return false;

    jclass cls = getClass(className);
    if(!cls) return false;

    JNINativeMethod *method = new JNINativeMethod[1];
    method[0].name = new char[methodName.length() + 1];
    method[0].signature = new char[signature.length() + 1];

    std::strcpy(method[0].name, methodName.c_str());
    std::strcpy(method[0].signature, signature.c_str());

    method[0].fnPtr = functionPointer;

    int res = env->RegisterNatives(cls, method, 1);
    if(res != JNI_OK)
    {
        displayJNIError("Cannot register native method", res);
        return false;
    }
    else
    {
        return true;
    }
}

bool Launcher::stopVM()
{
    if(!jvm)
    {
        std::cerr << "Cannot stop JVM, JVM is not started yet" << std::endl;
        return false;
    }
    jint res = jvm->DestroyJavaVM();
    displayJNIError("Stopping Java VM", res);

    if(res == JNI_OK)
    {
        jvm = nullptr;
        return true;
    }
    else
    {
        return false;
    }

}

bool Launcher::isAssignableFrom(std::string subclass, std::string superclass)
{
    JNIEnv* env = getEnv();
    if(!env) return false;

    jclass sub = getClass(subclass);
    jclass sup = getClass(superclass);

    if(sub && sup)
    {
        return env->IsAssignableFrom(sub, sup);
    }
    else
    {
        return false;
    }
}

Launcher::~Launcher()
{
//    if(jvm)
//        stopVM();

    delete vmArguments.options;
}


