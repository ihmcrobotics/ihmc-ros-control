#ifndef LAUNCHER_H
#define LAUNCHER_H
#include <jni.h>
#include <string>


struct JavaMethod
{
    jclass clazz;
    jmethodID methodID;

};

struct StaticJavaMethod
{
    jclass clazz;
    jmethodID methodID;

};


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

    JavaMethod* getJavaMethod(std::string className, std::string method, std::string signature);
    StaticJavaMethod* getStaticJavaMethod(std::string className, std::string method, std::string signature);

    void call(StaticJavaMethod*, ...);
    void call(JavaMethod*, jobject obj, ...);

    jobject createObject(JavaMethod* constructor, ...);

    bool registerNativeMethod(std::string className, std::string method, std::string signature, void* functionPointer);

    ~Launcher();
};

#endif // LAUNCHER_H
