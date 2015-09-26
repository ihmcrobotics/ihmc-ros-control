//
// Created by dstephen on 9/25/15.
//

#ifndef PROJECT_SIMPLEPDCONTROLLER_H
#define PROJECT_SIMPLEPDCONTROLLER_H


#include <hardware_interface/joint_command_interface.h>

namespace ihmc_ros_control {

    class SimplePDController {
    public:
        SimplePDController(hardware_interface::JointHandle jointHandle, bool isAngular, double kp, double kd);

        SimplePDController();

        virtual ~SimplePDController();

        virtual void updateJointHandle(double desiredPosition, double desiredVelocity);

        virtual void setKp(double newKp)
        {
            this->kp = newKp;
        }

        virtual void setKd(double newKd)
        {
            this->kd = newKd;
        }

    private:
        hardware_interface::JointHandle controlledJointHandle;
        bool isAngular;
        double kp;
        double kd;

        double error_p;
        double error_d;

        double action_p;
        double action_d;

        virtual void updateJointHandleLinear(double desiredPosition, double desiredVelocity);

        virtual void updateJointHandleAngular(double desiredPosition, double desiredVelocity);

        static double angleDifferenceMinusPiToPi(double firstAngle, double secondAngle)
        {
            double difference = firstAngle - secondAngle;
            difference = fmod(difference, (2.0 * M_PI));
            difference = shiftAngleToStartOfRange(difference, -M_PI);

            return difference;
        }

        static double shiftAngleToStartOfRange(double angleToShift, double startOfAngleRange)
        {
            double ret = angleToShift;
            double epsilon = 1e-10;
            startOfAngleRange = startOfAngleRange - epsilon;

            if(angleToShift < startOfAngleRange)
            {
                ret = angleToShift + ceil((startOfAngleRange - angleToShift) / (2.0 * M_PI)) * M_PI * 2.0;
            }

            if(angleToShift >= (startOfAngleRange + M_PI * 2.0))
            {
                ret = angleToShift - floor((angleToShift - startOfAngleRange) / (2.0 * M_PI)) * M_PI * 2.0;
            }

            return ret;
        }

    };

}


#endif //PROJECT_SIMPLEPDCONTROLLER_H
