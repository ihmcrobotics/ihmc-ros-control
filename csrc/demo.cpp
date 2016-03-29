
#include "jvmLauncher/launcher.h"
#include <iostream>
#include <jni.h>
#include <time.h>

JNIEXPORT void JNICALL callVoidFunctionWithString
  (JNIEnv *env, jobject obj, jstring str)
{
    const char * cstr = env->GetStringUTFChars(str, 0);
    std::cout << "Got string from java: " << cstr << std::endl;
    env->ReleaseStringUTFChars(str, cstr);
}

JNIEXPORT jint JNICALL callIntFunctionWithBoolean
  (JNIEnv *env, jobject obj, jboolean a, jboolean b)
{
    return (a?1000:100) + (b?10:1);
}


timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

int main(int argc, char *argv[])
{
    Launcher launcher("-Djava.class.path=src/ihmcroscontrol/bin");


    launcher.startVM();

    launcher.registerNativeMethod("us.ihmc.rosControl.launcher.TestJVMLaunchCallback", "callVoidFunctionWithString", "(Ljava/lang/String;)V", (void *)&callVoidFunctionWithString);
    launcher.registerNativeMethod("us.ihmc.rosControl.launcher.TestJVMLaunchCallback", "callIntFunctionWithBoolean", "(ZZ)I", (void *)&callIntFunctionWithBoolean);


    JavaMethod* ctor = launcher.getJavaMethod("us.ihmc.rosControl.launcher.TestJVMLaunchCallback", "<init>", "(I)V");
    JavaMethod* method = launcher.getJavaMethod("us.ihmc.rosControl.launcher.TestJVMLaunchCallback", "execute", "(I)V");

    JavaMethod* add = launcher.getJavaMethod("us.ihmc.rosControl.launcher.TestJVMLaunchCallback", "add", "()V");

    if(ctor && method && add)
    {
        jobject obj = launcher.createObject(ctor, 42);


        timespec start;
        timespec end;
        for(int c = 0; c < 10; c++)
        {
            clock_gettime(CLOCK_MONOTONIC, &start);
            for(int i = 0; i < 100000; i++)
            {
                launcher.call(add, obj);
            }
            clock_gettime(CLOCK_MONOTONIC, &end);

            timespec elapsed = diff(start, end);
            std::cout << "Took " << elapsed.tv_sec << "s, " << elapsed.tv_nsec << "nsec for 100000 iterations" << std::endl;

        }




        launcher.call(method, obj, 124);

    }




    launcher.stopVM();

}
