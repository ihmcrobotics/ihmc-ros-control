
#include <jvmLauncher/launcher.h>
#include <iostream>

int main(int argc, char *argv[])
{
    Launcher launcher("-verbose:gc -XX:+UseSerialGC");


    launcher.startVM();
    launcher.callStaticVoidMethod("java.lang.System", "gc");
    launcher.stopVM();
    launcher.callStaticVoidMethod("java.lang.System", "gc");
}
