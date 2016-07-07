#ifndef NATIVEJOINTHANDLEHOLDER_H
#define NATIVEJOINTHANDLEHOLDER_H

#include "NativeJointStateHandleHolder.h"
#include <hardware_interface/joint_command_interface.h>


namespace ihmc_ros_control
{
    class NativeJointHandleHolder : public NativeJointStateHandleHolder
    {
    public:
        NativeJointHandleHolder(hardware_interface::JointHandle handle);

        virtual ~NativeJointHandleHolder();

        void writeCommandIntoBuffer(int& index, double* buffer);

        int commandSize();

    private:
        hardware_interface::JointHandle handle;
    };
}

#endif // NATIVEJOINTHANDLEHOLDER_H
