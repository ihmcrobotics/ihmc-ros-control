//
// Created by dstephen on 9/25/15.
//

#include "SimplePDController.h"

namespace ihmc_ros_control {

    SimplePDController::SimplePDController(hardware_interface::JointHandle jointHandle, bool isAngular, double kp = 0.0, double kd = 0.0)
            : controlledJointHandle(jointHandle), isAngular(isAngular), kp(kp), kd(kd)
    { }

    SimplePDController::SimplePDController() { }

    SimplePDController::~SimplePDController() { }

    void SimplePDController::updateJointHandle(double desiredPosition, double desiredVelocity)
    {
        if(this->isAngular)
        {
            this->updateJointHandleAngular(desiredPosition, desiredVelocity);
        }
        else
        {
            this->updateJointHandleLinear(desiredPosition, desiredVelocity);
        }
    }

    void SimplePDController::updateJointHandleLinear(double desiredPosition, double desiredVelocity)
    {
        this->error_p = desiredPosition - this->controlledJointHandle.getPosition();
        this->error_d = desiredVelocity - this->controlledJointHandle.getVelocity();

        this->action_p = this->kp * this->error_p;
        this->action_d = this->kd * this->error_d;

        this->controlledJointHandle.setCommand(this->action_p + this->action_d);
    }

    void SimplePDController::updateJointHandleAngular(double desiredPosition, double desiredVelocity)
    {
        this->error_p = angleDifferenceMinusPiToPi(desiredPosition, this->controlledJointHandle.getPosition());
        this->error_d = desiredVelocity - this->controlledJointHandle.getVelocity();

        this->action_p = this->kp * this->error_p;
        this->action_d = this->kd * this->error_d;

        this->controlledJointHandle.setCommand(this->action_p + this->action_d);
    }
}