This library implements a general interface for controlling robots.


## Installation

This is an interface library, i.e. it is header-only. There are multiple ways of using it in your project. 

### Dependencies

- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [sackmesser](https://gitlab.com/tloew/sackmesser)

### Subdirectory

Clone this repository directly into your project tree and add

    add_subdirectory(orwell)

to your CMakeLists.txt

and then add

    target_link_libraries(TARGET PUBLIC orwell)

### System

This library can be installed system-wide using the following list of commands:

    mkdir build && cd build
    cmake ..
    make
    make install

Afterwards *orwell* can be used by adding

    find_package(orwell REQUIRED)
    target_link_libraries(TARGET PUBLIC orwell)

to your CMakeLists.txt

### ROS

This repository contains [package.xml](package.xml), so it can also be placed directly in a ROS workspace.

Afterwards *orwell* can be used by other packages in the workspace by adding

    find_package(orwell REQUIRED)
    target_link_libraries(TARGET PUBLIC orwell)

to your CMakeLists.txt

## Usage

This library is agnostic of the kinematics/dynamics library that is being used. Therefore, in order to use it, you must provide an interface to your desired kinematics/dynamics library. This is achieved by deriving from [RobotModel](src/orwell/RobotModel.hpp). 

Then assuming you have an instance of RobotModel and sackmesser::Interface a controller manager should be created:

    orwell::ControllerManager<7> controller_manager(interface, robot_model);

This will load controllers according to the provided configuration file. The desired controller can then be loaded via get methods, e.g. 

    auto controller = controller_manager.getTorqueController("CONTROLLER_NAME");

This controller should then be placed in your control loop with calls to getControlCommand, i.e.

    controller->getControlCommand(robot_state);

where robot_state is an instance of [RobotState](src/orwell/RobotState.hpp), which contains the current robot state.

### Documentation 

The full library documentation can be found [here](http://tobiloew.ch/software/orwell/documentation/index.html).