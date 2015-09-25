
#include <jvmLauncher/launcher.h>
#include <iostream>
#include <jni.h>

JNIEXPORT void JNICALL callVoidFunctionWithString
  (JNIEnv *env, jobject clazz, jstring str)
{
    const char * cstr = env->GetStringUTFChars(str, 0);
    std::cout << "Got string from java: " << cstr << std::endl;
    env->ReleaseStringUTFChars(str, cstr);
}

JNIEXPORT jint JNICALL callIntFunctionWithBoolean
  (JNIEnv *env, jobject clazz, jboolean a, jboolean b)
{
    return (a?1000:100) + (b?10:1);
}


int main(int argc, char *argv[])
{
    Launcher launcher("-Djava.class.path=src/ihmcroscontrol/build/classes/main");


    launcher.startVM();

    launcher.registerNativeMethod("us.ihmc.rosControl.launcher.TestJVMLaunchCallback", "callVoidFunctionWithString", "(Ljava/lang/String;)V", (void *)&callVoidFunctionWithString);
    launcher.registerNativeMethod("us.ihmc.rosControl.launcher.TestJVMLaunchCallback", "callIntFunctionWithBoolean", "(ZZ)I", (void *)&callIntFunctionWithBoolean);


    JavaMethod* ctor = launcher.getJavaMethod("us.ihmc.rosControl.launcher.TestJVMLaunchCallback", "<init>", "(I)V");
    JavaMethod* method = launcher.getJavaMethod("us.ihmc.rosControl.launcher.TestJVMLaunchCallback", "execute", "(I)V");

    if(ctor && method)
    {
        jobject obj = launcher.createObject(ctor, 42);
        launcher.call(method, obj, 124);
    }




    launcher.stopVM();

}
