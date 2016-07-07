#ifndef NATIVEJOINTSTATEHANDLEHOLDER_H
#define NATIVEJOINTSTATEHANDLEHOLDER_H

#include "NativeUpdateableInterface.h"
#include <hardware_interface/joint_state_interface.h>


namespace ihmc_ros_control
{
    class NativeJointStateHandleHolder : public NativeUpdateableInterface
    {
    public:
        NativeJointStateHandleHolder(hardware_interface::JointStateHandle handle);

        virtual ~NativeJointStateHandleHolder();

        void readStateIntoBuffer(int& index, double* buffer);
        virtual void writeCommandIntoBuffer(int& index, double* buffer);

        int stateSize();
        virtual int commandSize();

    private:
        hardware_interface::JointStateHandle handle;
    };
}

#endif // NATIVEJOINTSTATEHANDLEHOLDER_H
