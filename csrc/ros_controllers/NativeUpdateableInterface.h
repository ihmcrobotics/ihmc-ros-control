#ifndef NATIVEUPDATABLEINTERFACE_H
#define NATIVEUPDATABLEINTERFACE_H

namespace ihmc_ros_control
{
    class NativeUpdateableInterface
    {
    public:
        virtual ~NativeUpdateableInterface() {}

        virtual void readStateIntoBuffer(int& index, double* buffer) = 0;
        virtual void writeCommandIntoBuffer(int& index, double* buffer) = 0;

        virtual int stateSize() = 0;
        virtual int commandSize() = 0;
    };
}

#endif // NATIVEUPDATABLEINTERFACE_H
