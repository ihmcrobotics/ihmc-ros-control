//
// Created by dstephen on 9/25/15.
//

#ifndef PROJECT_VALKYRIEFORWARDCOMMANDCONTROLLER_H
#define PROJECT_VALKYRIEFORWARDCOMMANDCONTROLLER_H

#include <unordered_map>
#include <pluginlib/class_list_macros.h>
#include "val_controller_interface/controller.h"
#include "../rrbot_controllers/SimplePDController.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>

namespace ihmc_ros_control {
    class ValkyrieForwardCommandController : public val_controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        ValkyrieForwardCommandController();
        virtual ~ValkyrieForwardCommandController();

        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& n) override;
        void starting(const ros::Time& time) override;
        void update(const ros::Time& time, const ros::Duration& period) override;

    private:
//        std::vector< hardware_interface::JointHandle > jointHandles;
        std::unordered_map<std::string, hardware_interface::JointHandle> jointHandles;
        std::vector<hardware_interface::ImuSensorHandle> imuHandles;
        std::vector<hardware_interface::ForceTorqueSensorHandle> forceTorqueHandles;

        bool configureJointHandles(hardware_interface::RobotHW *robot_hw, const ros::NodeHandle &n);

        void configureIMUHandles(hardware_interface::RobotHW *robot_hw, const ros::NodeHandle &n);

        bool configureForceTorqueHandles(hardware_interface::RobotHW *robot_hw, const ros::NodeHandle &n);

        SimplePDController* leftKneePDController;

        double leftKneePositionCommand;
        double directionFactor;
    };
}

#endif //PROJECT_VALKYRIEFORWARDCOMMANDCONTROLLER_H
