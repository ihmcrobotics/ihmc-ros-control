#ifndef IMUHANDLEHOLDER_H
#define IMUHANDLEHOLDER_H

#include "NativeJointHandleHolder.h"
#include <hardware_interface/imu_sensor_interface.h>

namespace ihmc_ros_control
{
    const int orientationSize = 4;
    const int covarianceSize = 9;
    const int angularVelocitySize = 3;
    const int linearAccelerationSize = 3;

    class NativeIMUHandleHolder : public NativeUpdateableInterface
    {
    public:
        NativeIMUHandleHolder();

        NativeIMUHandleHolder(hardware_interface::ImuSensorHandle handle);

        virtual ~NativeIMUHandleHolder();

        void readStateIntoBuffer(int& index, double* buffer);
        void writeCommandIntoBuffer(int& index, double* buffer);

        int stateSize();
        int commandSize();

    private:
        hardware_interface::ImuSensorHandle handle;
    };
}
#endif // IMUHANDLEHOLDER_H
