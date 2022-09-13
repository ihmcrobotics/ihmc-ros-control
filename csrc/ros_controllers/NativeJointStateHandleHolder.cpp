#include "NativeJointStateHandleHolder.h"


namespace ihmc_ros_control
{
    NativeJointStateHandleHolder::NativeJointStateHandleHolder(hardware_interface::JointStateHandle handle) :
        handle(handle)
    {
    }

    NativeJointStateHandleHolder::~NativeJointStateHandleHolder()
    {

    }

    void NativeJointStateHandleHolder::readStateIntoBuffer(int& index, double* buffer)
    {
        buffer[index++] = handle.getEffort();
        buffer[index++] = handle.getPosition();
        buffer[index++] = handle.getVelocity();
    }

    int NativeJointStateHandleHolder::stateSize()
    {
        return 3;
    }

    int NativeJointStateHandleHolder::commandSize()
    {
        return 0;
    }

    void NativeJointStateHandleHolder::writeCommandIntoBuffer(int &index, double *buffer)
    {
        // do nothing
    }

}
