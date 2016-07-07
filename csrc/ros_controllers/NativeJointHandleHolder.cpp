#include "NativeJointHandleHolder.h"


namespace ihmc_ros_control
{
    NativeJointHandleHolder::NativeJointHandleHolder(hardware_interface::JointHandle handle) : NativeJointStateHandleHolder(handle),
        handle(handle)
    {
    }

    NativeJointHandleHolder::~NativeJointHandleHolder()
    {

    }

    void NativeJointHandleHolder::writeCommandIntoBuffer(int& index, double* buffer)
    {
        handle.setCommand(buffer[index++]);
    }

    int NativeJointHandleHolder::commandSize()
    {
        return 1;
    }

}
