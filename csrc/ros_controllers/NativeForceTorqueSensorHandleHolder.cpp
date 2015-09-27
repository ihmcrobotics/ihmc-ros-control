#include "NativeForceTorqueSensorHandleHolder.h"


namespace ihmc_ros_control
{
    NativeForceTorqueSensorHandleHolder::NativeForceTorqueSensorHandleHolder(hardware_interface::ForceTorqueSensorHandle handle) :
        handle(handle)
    {
    }

    NativeForceTorqueSensorHandleHolder::~NativeForceTorqueSensorHandleHolder()
    {

    }

    void NativeForceTorqueSensorHandleHolder::readStateIntoBuffer(int& index, double* buffer)
    {
        for(int i = 0; i < forceLength; i++)
        {
            buffer[index++] = handle.getForce()[i];
        }

        for(int i = 0; i < torqueLength; i++)
        {
            buffer[index++] = handle.getTorque()[i];
        }
    }

    void NativeForceTorqueSensorHandleHolder::writeCommandIntoBuffer(int& index, double* buffer)
    {
    }

    int NativeForceTorqueSensorHandleHolder::stateSize()
    {
        return forceLength + torqueLength;
    }

    int NativeForceTorqueSensorHandleHolder::commandSize()
    {
        return 0;
    }


}
