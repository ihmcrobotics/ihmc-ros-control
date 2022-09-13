#include "NativeJointGainsHandleHolder.h"


namespace ihmc_ros_control
{
    NativeJointGainsHandleHolder::NativeJointGainsHandleHolder(hardware_interface::JointGainsHandle handle) : NativeUpdateableInterface(),
        handle(handle)
    {
    }

    NativeJointGainsHandleHolder::~NativeJointGainsHandleHolder()
    {

    }

    void NativeJointGainsHandleHolder::writeCommandIntoBuffer(int& index, double* buffer)
    {
        handle.setStiffnessCommand(buffer[index++]);
        handle.setDampingCommand(buffer[index++]);
    }

    void NativeJointGainsHandleHolder::readStateIntoBuffer(int& index, double* buffer)
    {
        // Nothing to do here
    }

    int NativeJointGainsHandleHolder::stateSize()
    {
        return 0;
    }

    int NativeJointGainsHandleHolder::commandSize()
    {
        return 2;
    }

}
