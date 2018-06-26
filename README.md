# IHMC ROS Control Support

Provides a bridge between Java and ROS Control. The simple interface, `ihmc_ros_control::IHMCRosControlJavaBridge` and the similarly named Java interface, provides a way to write controllers that operate on the resources exposed by `ros_control`'s `hardware_interface::EffortJointInterface`. `ihmc_ros_control::IHMCWholeRobotControlJavaBridge` is an extension of the base interface and allows for implementing a `ros_control` controller that hooks in to the `initRequest` method to acquire resources of multiple types in a single ROS plugin. Currently, you can use this interface to write controllers that work with all of the following simultaneously:

- `hardware_interface::JointStateInterface`
- `hardware_interface::EffortJointInterface`
- `hardware_interface::PositionJointInterface`
- `hardware_interface::ImuSensorInterface`
- `hardware_interface::ForceTorqueSensorInterface`

## Installation

`ihmc_ros_control` is a Catkin package. We currently support ROS Indigo on Ubuntu 14.04 LTS with release 0.5.0 and ROS Kinetic on Ubuntu 16.04 LTS with release 0.6.0. We plan to continue targeting LTS releases of both ROS and Ubuntu.

We use some C++11 features in `ihmc_ros_control`. The versions of gcc and clang available in Ubuntu 14.04 should be fine, and we have modified our CMakeLists.txt accordingly
to set up the C++11 features correctly. You will also need a Java JDK (not JRE), as you will need the Java Native Interface headers to compile the C++ code. We support OpenJDK 8 and Oracle JDK 8.

On Ubuntu 16.04:

```bash
$ sudo apt-get update
$ sudo apt-get install openjdk-8-jdk
```

Ubuntu 14.04 may require tapping a PPA: https://launchpad.net/~openjdk-r/+archive/ubuntu/ppa

To install `ihmc-ros-control`, simply clone the repository in to a catkin workspace and then `catkin_make install` or `catkin build && catkin install` if using `catkin_tools`.
If you have an existing catkin workspace, we recommend "overlaying" any IHMC code on top of your code in its own workspace, e.g. `ihmc_catkin_ws`. If you don't currently have any IHMC code, that might look something like:

```bash
$ cd ~
$ mkdir -p ihmc_catkin_ws/src
$ cd ihmc_catkin_ws
$ catkin_init_workspace src
$ cd src
$ # use https://github.com/ihmcrobotics/ihmc-ros-control.git if you don't have ssh keys configured on GitHub
$ git clone git@github.com:ihmcrobotics/ihmc-ros-control.git
$ cd ..
$ catkin_make install
```

You may also be interested in [ihmc_workspaces](https://github.com/ihmcrobotics/ihmc_workspaces) which provides ways to get at other IHMC ROS packages.

### CMake JNI Errors
If CMake has problems finding the JNI packages or if there are Java/JNI related compilation problems during `catkin_make`, export JAVA_HOME before running `catkin_make`:

```bash
$ export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64/
```

## Writing controllers (Java)

IHMCRosControl is provided as a Maven artifact on Bintray: https://bintray.com/ihmcrobotics/maven-release/IHMCRosControl. You can find instructions for depending on our Maven artifacts at Bintray.

As a quick example, to use IHMCRosControl in a Gradle project:

- Add the following repository to your buildscript:
```gradle
repositories {
    maven {
        url  "http://dl.bintray.com/ihmcrobotics/maven-release"
    }
}
```
- Add the following dependency to your gradle dependencies:
```gradle
compile group: 'us.ihmc', name: 'IHMCRosControl', version: '0.6.0'
```

Then implement either "IHMCRosControlJavaBridge" or "IHMCWholeRobotControlJavaBridge", depending on which hardware interfaces you would like to control.

## Examples

This package provides an example launch file for controlling the RRBot Gazebo demo via the simple EffortJointInterface adapter. For more complex examples, including an example of the Whole Robot interface, take a look at [the Valkyrie project in IHMC Open Robotics Software](https://github.com/ihmcrobotics/ihmc-open-robotics-software) along with the [ihmc_valkyrie_ros](https://github.com/ihmcrobotics/ihmc_valkyrie_ros) ROS package for launch files.
