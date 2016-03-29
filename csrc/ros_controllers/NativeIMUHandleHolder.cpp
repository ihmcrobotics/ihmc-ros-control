#include "NativeIMUHandleHolder.h"


namespace ihmc_ros_control
{
    NativeIMUHandleHolder::NativeIMUHandleHolder(hardware_interface::ImuSensorHandle handle) :
        handle(handle)
    {
    }


    NativeIMUHandleHolder::~NativeIMUHandleHolder()
    {

    }

    void NativeIMUHandleHolder::readStateIntoBuffer(int& index, double* buffer)
    {
        for(int i = 0; i < orientationSize; i++)
        {
            buffer[index++] = handle.getOrientation()[i];
        }

        for(int i = 0; i < covarianceSize; i++)
        {
            buffer[index++] = handle.getOrientationCovariance()[i];
        }
        for(int i = 0; i < angularVelocitySize; i++)
        {
            buffer[index++] = handle.getAngularVelocity()[i];
        }

        for(int i = 0; i < covarianceSize; i++)
        {
            buffer[index++] = handle.getAngularVelocityCovariance()[i];
        }
        for(int i = 0; i < linearAccelerationSize; i++)
        {
            buffer[index++] = handle.getLinearAcceleration()[i];
        }

        for(int i = 0; i < covarianceSize; i++)
        {
            buffer[index++] = handle.getLinearAccelerationCovariance()[i];
        }
    }

    void NativeIMUHandleHolder::writeCommandIntoBuffer(int& index, double* buffer)
    {
        // Nothing to do here
    }

    int NativeIMUHandleHolder::stateSize()
    {
        return orientationSize + angularVelocitySize + linearAccelerationSize +  3 * covarianceSize;
    }

    int NativeIMUHandleHolder::commandSize()
    {
        return 0;
    }
}
