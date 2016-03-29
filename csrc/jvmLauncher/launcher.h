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
    JavaVMInitArgs vmArguments;
    void displayJNIError(std::string prefix, int error);

    jclass getClass(std::string className);

public:
    Launcher(std::string vmArguments);
    bool startVM(std::string workingDirectory = ".");
    bool stopVM();

    JavaMethod* getJavaMethod(std::string className, std::string method, std::string signature);
    StaticJavaMethod* getStaticJavaMethod(std::string className, std::string method, std::string signature);

    void call(StaticJavaMethod*, ...);
    void call(JavaMethod*, jobject obj, ...);
    bool callBooleanMethod(JavaMethod*, jobject obj, ...);

    jobject createObject(JavaMethod* constructor, ...);
    void release(jobject object);
    void release(StaticJavaMethod* method);
    void release(JavaMethod* method);

    bool registerNativeMethod(std::string className, std::string method, std::string signature, void* functionPointer);

    bool isAssignableFrom(std::string subclass, std::string superclass);

    JNIEnv* getEnv();

    void attachCurrentThread();
    void detachCurrentThread();

    ~Launcher();
};

#endif // LAUNCHER_H
