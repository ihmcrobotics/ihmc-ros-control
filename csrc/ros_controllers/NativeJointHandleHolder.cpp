#include "NativeJointHandleHolder.h"


namespace ihmc_ros_control
{
    NativeJointHandleHolder::NativeJointHandleHolder(hardware_interface::JointHandle handle) :
        handle(handle)
    {
    }

    NativeJointHandleHolder::~NativeJointHandleHolder()
    {

    }

    void NativeJointHandleHolder::readStateIntoBuffer(int& index, double* buffer)
    {
        buffer[index++] = handle.getEffort();
        buffer[index++] = handle.getPosition();
        buffer[index++] = handle.getVelocity();
    }

    void NativeJointHandleHolder::writeCommandIntoBuffer(int& index, double* buffer)
    {
        handle.setCommand(buffer[index++]);
    }

    int NativeJointHandleHolder::stateSize()
    {
        return 3;
    }

    int NativeJointHandleHolder::commandSize()
    {
        return 1;
    }

}
