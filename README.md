# IHMC ROS Control Support

Provides a bridge between Java and ROS Control.

## Installation

Checkout in your catkin_ws/src directory and run catkin_make in catkin_ws.

## Writing controllers

Add the following dependency to your gradle dependencies

compile group: 'us.ihmc', name: 'IHMCRosControl', version: '0.1'

Implement either "IHMCRosControlJavaBridge" or "IHMCValkyrieControlJavaBridge", depending on if you want to control only joints or have access to the IMU and force torque sensors as well.

## Running

Pattern match launch/java_control.launch or launch/val_java_control.launch depending if you used "IHMCRosControlJavaBridge" or "IHMCValkyrieControlJavaBridge" respectively.

Setup your configuration based on config/jvm_param.yaml or config/val_java_param.yaml, depending on the launch file. Make sure you setup the working directory and classpath correctly. 


 