#ifndef NATIVEJOINTHANDLEHOLDER_H
#define NATIVEJOINTHANDLEHOLDER_H

#include "NativeUpdateableInterface.h"
#include <hardware_interface/joint_command_interface.h>


namespace ihmc_ros_control
{
    class NativeJointHandleHolder : public NativeUpdateableInterface
    {
    public:
        NativeJointHandleHolder(hardware_interface::JointHandle handle);

        virtual ~NativeJointHandleHolder();

        void readStateIntoBuffer(int& index, double* buffer);
        void writeCommandIntoBuffer(int& index, double* buffer);

        int stateSize();
        int commandSize();

    private:
        hardware_interface::JointHandle handle;
    };
}

#endif // NATIVEJOINTHANDLEHOLDER_H
