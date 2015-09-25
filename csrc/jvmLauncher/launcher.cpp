#include <jvmLauncher/launcher.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include <algorithm>

#include <jni.h>
#include <vector>

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

void Launcher::call(JavaMethod *method, jobject obj, ...)
{
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
        std::cout << javaOptions[i].optionString << "::" << std::endl;
    }

    vmArguments.nOptions = 1;
    vmArguments.options = javaOptions;
    vmArguments.ignoreUnrecognized = true;
}

bool Launcher::startVM()
{
    jint res = JNI_CreateJavaVM(&jvm, (void**) &env, &vmArguments);
    displayJNIError("Starting Java VM", res);
    return res == JNI_OK;
}

jclass Launcher::getClass(std::string className)
{
    std::string classNameCopy(className);
    std::replace (classNameCopy.begin(), classNameCopy.end(), '.', '/');

    jclass cls = env->FindClass(classNameCopy.c_str());
    if(!cls)
    {
        std::cerr << "Cannot find class " << classNameCopy << std::endl;
        return NULL;
    }
    return cls;
}

StaticJavaMethod* Launcher::getStaticJavaMethod(std::string className, std::string methodName, std::string signature)
{

    if(!env)
    {
        std::cerr << "JVM is not started" << std::endl;
        return nullptr;
    }

    jclass cls = getClass(className);
    if(!cls) return nullptr;


    jmethodID mid = env->GetStaticMethodID(cls, methodName.c_str(), signature.c_str());
    if(!mid)
    {
        std::cerr << "Cannot find method " << methodName << "()" << std::endl;
        return nullptr;
    }

    StaticJavaMethod* method = new StaticJavaMethod();
    method->clazz = cls;
    method->methodID = mid;

    return method;
}

JavaMethod* Launcher::getJavaMethod(std::string className, std::string methodName, std::string signature)
{

    if(!env)
    {
        std::cerr << "JVM is not started" << std::endl;
        return nullptr;
    }

    jclass cls = getClass(className);
    if(!cls) return nullptr;


    jmethodID mid = env->GetMethodID(cls, methodName.c_str(), signature.c_str());
    if(!mid)
    {
        std::cerr << "Cannot find method " << methodName << "()" << std::endl;
        return nullptr;
    }

    JavaMethod* method = new JavaMethod();
    method->clazz = cls;
    method->methodID = mid;

    return method;
}

bool Launcher::registerNativeMethod(std::string className, std::string methodName, std::string signature, void *functionPointer)
{
    if(!env)
    {
        std::cerr << "JVM is not started" << std::endl;
        return false;
    }

    jclass cls = getClass(className);
    if(!cls) return false;

    JNINativeMethod *method = new JNINativeMethod[1];
    method[0].name = new char[methodName.length() + 1];
    method[0].signature = new char[signature.length() + 1];

    std::strcpy(method[0].name, methodName.c_str());
    std::strcpy(method[0].signature, signature.c_str());

    method[0].fnPtr = functionPointer;

    env->RegisterNatives(cls, method, 1);
    return true;
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
        env = NULL;
        jvm = NULL;
        return true;
    }
    else
    {
        return false;
    }

}

Launcher::~Launcher()
{
    if(jvm)
        stopVM();

    delete vmArguments.options;
}

