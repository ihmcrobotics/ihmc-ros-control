//
// Created by dstephen on 9/25/15.
//

#include "ValkyrieForwardCommandController.h"

namespace ihmc_ros_control
{
    ValkyrieForwardCommandController::ValkyrieForwardCommandController()
    {
        this->leftKneePositionCommand = 0.0;
        this->directionFactor = 1.0;
    }

    ValkyrieForwardCommandController::~ValkyrieForwardCommandController() { }

    bool ValkyrieForwardCommandController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &n) {

        this->jointHandles.clear();
        this->imuHandles.clear();
        this->forceTorqueHandles.clear();

//        configureIMUHandles(robot_hw, n);

        bool configurationSuccessful= configureJointHandles(robot_hw, n);

        if(!configurationSuccessful)
        {
            return false;
        }

//        configurationSuccessful = configureForceTorqueHandles(robot_hw, n);

        return configurationSuccessful;

    }

    bool ValkyrieForwardCommandController::configureForceTorqueHandles(hardware_interface::RobotHW *robot_hw,
                                                                       const ros::NodeHandle &n) {
        bool configurationSuccessful = true;
        std::vector<std::string> forceTorqueNames;
        if (n.getParam("force_torques", forceTorqueNames))
        {
            hardware_interface::ForceTorqueSensorInterface* forceTorqueSensorInterface = robot_hw->get<hardware_interface::ForceTorqueSensorInterface>();

            if (forceTorqueSensorInterface)
            {
                for (unsigned int i = 0; i < forceTorqueNames.size(); ++i)
                {
                    forceTorqueHandles.push_back(forceTorqueSensorInterface->getHandle(forceTorqueNames[i]));
                }
            }
            else
            {
                ROS_ERROR("Could not find ForceTorqueSensorInterface in RobotHW.");
            }
        }
        else
        {
            ROS_ERROR("No force_torques given (namespace: %s)", n.getNamespace().c_str());
            configurationSuccessful = false;
        }
        return configurationSuccessful;
    }

    void ValkyrieForwardCommandController::configureIMUHandles(hardware_interface::RobotHW *robot_hw,
                                                               const ros::NodeHandle &n) {
        std::vector<std::string> imuNames;
        if (n.getParam("imus", imuNames))
        {
            hardware_interface::ImuSensorInterface* imuSensorInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();

            if (imuSensorInterface)
            {
                for (unsigned int i = 0; i < imuNames.size(); ++i)
                {
                    imuHandles.push_back(imuSensorInterface->getHandle(imuNames[i]));
                }
            }
            else
            {
                ROS_ERROR("Could not find ImuSensorInterface in RobotHW.");
            }
        }
        else
        {
            ROS_WARN("No imus given (namespace: %s)", n.getNamespace().c_str());
        }
    }

    bool ValkyrieForwardCommandController::configureJointHandles(hardware_interface::RobotHW *robot_hw,
                                                                 const ros::NodeHandle &n) {

        hardware_interface::EffortJointInterface* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
        const hardware_interface::JointHandle &jointHandle = effortJointInterface->getHandle(std::string("leftKneePitch"));
        ROS_INFO("Joint handle: %s", jointHandle.getName().c_str());
//        effortJointInterface->claim(std::string("leftKneePitch"));
        jointHandles.insert(std::pair<std::string, hardware_interface::JointHandle>(std::string("leftKneePitch"), jointHandle));
        return true;

//        bool configurationSuccessful = true;
//        std::vector<std::string> jointNames;
//        if (n.getParam("joints", jointNames))
//        {
//            hardware_interface::EffortJointInterface* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
//
//            if (effortJointInterface)
//            {
//                effortJointInterface->clearClaims();
//                for (unsigned int i = 0; i < jointNames.size(); ++i)
//                {
//                    const hardware_interface::JointHandle &jointHandle = effortJointInterface->getHandle(jointNames[i]);
//                    effortJointInterface->claim(jointNames[i]);
//                    jointHandles.insert(std::pair<std::string, hardware_interface::JointHandle>(jointNames[i], jointHandle));
//                }
//            }
//            else
//            {
//                ROS_ERROR("Could not find EffortJointInterface in RobotHW.");
//                configurationSuccessful = false;   //! joints are required
//            }
//        }
//        else
//        {
//            ROS_ERROR("No joints given (namespace: %s)", n.getNamespace().c_str());
//            configurationSuccessful = false;   //! joints are required
//        }
//        return configurationSuccessful;
    }

    void ValkyrieForwardCommandController::starting(const ros::Time &time)
    {
        hardware_interface::JointHandle leftKneeJointHandle = jointHandles.at(std::string("leftKneePitch"));
        ROS_INFO("Starting, joint handle: %s", leftKneeJointHandle.getName().c_str());
        this->leftKneePDController = new SimplePDController(leftKneeJointHandle, true, 1000.0, 10.0);
    }

    void ValkyrieForwardCommandController::update(const ros::Time &time, const ros::Duration &period)
    {
        this->leftKneePositionCommand += (0.001 * this->directionFactor);

        if (this->leftKneePositionCommand <= 0.0) {
            this->directionFactor = 1.0;
        }
        else if (this->leftKneePositionCommand > 1.0) {
            this->directionFactor = -1.0;
        }

        this->leftKneePDController->updateJointHandle(this->leftKneePositionCommand, 0.0);
    }
}

PLUGINLIB_EXPORT_CLASS(
        ihmc_ros_control::ValkyrieForwardCommandController,
        controller_interface::ControllerBase)