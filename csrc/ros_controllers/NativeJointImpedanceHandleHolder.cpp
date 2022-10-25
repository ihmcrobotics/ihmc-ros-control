#include "NativeJointImpedanceHandleHolder.h"


namespace ihmc_ros_control
{
    NativeJointImpedanceHandleHolder::NativeJointImpedanceHandleHolder(hardware_interface::JointImpedanceHandle handle) : NativeUpdateableInterface(),
        handle(handle)
    {
    }

    NativeJointImpedanceHandleHolder::~NativeJointImpedanceHandleHolder()
    {

    }

    void NativeJointImpedanceHandleHolder::writeCommandIntoBuffer(int& index, double* buffer)
    {
        handle.setStiffnessCommand(buffer[index++]);
        handle.setDampingCommand(buffer[index++]);
        handle.setPositionCommand(buffer[index++]);
        handle.setVelocityCommand(buffer[index++]);
    }

    void NativeJointImpedanceHandleHolder::readStateIntoBuffer(int& index, double* buffer)
    {
        // Nothing to do here
    }

    int NativeJointImpedanceHandleHolder::stateSize()
    {
        return 0;
    }

    int NativeJointImpedanceHandleHolder::commandSize()
    {
        return 4;
    }

}
