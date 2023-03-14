#ifndef NATIVEJOINTGAINSHANDLEHOLDER_H
#define NATIVEJOINTGAINSHANDLEHOLDER_H

#include "NativeUpdateableInterface.h"
#include <hardware_interface/joint_command_interface.h>
#include "val_robot_interface/JointImpedanceInterface.hpp"


namespace ihmc_ros_control
{
    class NativeJointImpedanceHandleHolder : public NativeUpdateableInterface
    {
    public:
        NativeJointImpedanceHandleHolder(hardware_interface::JointImpedanceHandle handle);

        virtual ~NativeJointImpedanceHandleHolder();

        // Write the command from IHMC to the joint
        void writeCommandIntoBuffer(int& index, double* buffer);

        // Read state data from the buffer
        void readStateIntoBuffer(int& index, double* buffer);

        int stateSize();
        int commandSize();

    private:
        hardware_interface::JointImpedanceHandle handle;
    };
}

#endif // NATIVEJOINTGAINSHANDLEHOLDER_H
