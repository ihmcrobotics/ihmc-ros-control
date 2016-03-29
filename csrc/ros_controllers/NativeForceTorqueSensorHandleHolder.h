#ifndef NATIVEFORCETORQUESENSORHANDLEHOLDER_H
#define NATIVEFORCETORQUESENSORHANDLEHOLDER_H

#include "NativeJointHandleHolder.h"
#include <hardware_interface/force_torque_sensor_interface.h>

namespace ihmc_ros_control
{
    const int forceLength = 3;
    const int torqueLength = 3;

    class NativeForceTorqueSensorHandleHolder : public NativeUpdateableInterface
    {
    public:
        NativeForceTorqueSensorHandleHolder();

        NativeForceTorqueSensorHandleHolder(hardware_interface::ForceTorqueSensorHandle handle);

        virtual ~NativeForceTorqueSensorHandleHolder();

        void readStateIntoBuffer(int& index, double* buffer);
        void writeCommandIntoBuffer(int& index, double* buffer);

        int stateSize();
        int commandSize();

    private:
        hardware_interface::ForceTorqueSensorHandle handle;
    };
}
#endif // NATIVEFORCETORQUESENSORHANDLEHOLDER_H
