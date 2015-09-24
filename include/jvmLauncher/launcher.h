#ifndef LAUNCHER_H
#define LAUNCHER_H
#include <jni.h>
#include <string>

class Launcher
{
private:
    JavaVM *jvm;
    JNIEnv *env;
    JavaVMInitArgs vmArguments;
    void displayJNIError(std::string prefix, int error);

    jclass getClass(std::string className);

public:
    Launcher(std::string vmArguments);
    bool startVM();
    bool stopVM();

    bool callStaticVoidMethod(std::string className, std::string method);
    bool registerNativeMethod(std::string className, std::string method, std::string signature, void* functionPointer);

    ~Launcher();
};

#endif // LAUNCHER_H
