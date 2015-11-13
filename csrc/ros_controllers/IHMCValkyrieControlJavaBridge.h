#ifndef IHMCVALKYRIECONTROLJAVABRIDGE_H
#define IHMCVALKYRIECONTROLJAVABRIDGE_H

#include "IHMCRosControlJavaBridge.h"

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>

namespace ihmc_ros_control
{
    const std::string valkyrieControlInterfaceClass = "us.ihmc.rosControl.valkyrie.IHMCValkyrieControlJavaBridge";


    class IHMCValkyrieControlJavaBridge :
            public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        IHMCValkyrieControlJavaBridge();
        virtual ~IHMCValkyrieControlJavaBridge();


        void starting(const ros::Time& time) override;
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time&);

        /**
         * @brief addIMUToBuffer Add an IMU to the buffers, called from Java init()
         * @param imuName
         * @return
         */
        bool addIMUToBuffer(std::string imuName);

        /**
         * @brief addForceTorqueSensorToBuffer Add a force torque sensor to the buffers, called from Java init()
         * @param forceTorqueSensorName
         * @return
         */
        bool addForceTorqueSensorToBuffer(std::string forceTorqueSensorName);

    protected:
        virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh,
                                 std::set<std::string>& claimed_resources) override;


    private:
        IHMCRosControlJavaBridge ihmcRosControlJavaBridge;
        hardware_interface::ImuSensorInterface* imuSensorInterface;
        hardware_interface::ForceTorqueSensorInterface* forceTorqueSensorInterface;
    };

}

#endif // IHMCVALKYRIECONTROLJAVABRIDGE_H
