---
title: "OmniHex"
date: 2021-09-01T16:37:33+08:00
draft: false
description: Adaptive Nonlinear Model Predictive Control for An Overactuated Hexacopter
image: cover.png
comments: true
license: false
hidden: false
categories:
    - Ongoing_Project
tags:
    - UAV
    - MPC
    - ADAPTIVE CTRL
---

---
## INFO

> This page is WIP, as translating the original version of my research proposal is no easier than rewriting one in English. I also need to re-render charts and graphs with Chinese characters, which requires quite a lot of effort. Please expect this page to be complete by Feb 2022. Best regards~

---
## LINKS

* [Presentation of Previous Work (English)](https://github.com/ErcBunny/sharedDocs/raw/main/Project%20Experience.pptx)
* [Research Proposal (Chinese)](https://github.com/ErcBunny/sharedDocs/raw/main/omnihex.pdf)

---

> Repository readme below

# omniHex

![fooimg](https://ercbunny.github.io/p/omnihex/cover_huf54f83ec7f48f4593ffcb0ac862986d3_1368328_1600x0_resize_box_3.png)

[toc]

## Environment Setup

1. `Ubuntu 20.04 LTS` with `ROS2 foxy` configuration is recommended.
2. Install ROS2 [via debian](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) and [additional tools](https://docs.px4.io/master/en/ros/ros2_comm.html#install-ros-2).
3. Install Fast-RTPS-Gen using source code.
   * Install [Gradle v6.3](https://gradle.org/install/) through [sdkman](https://sdkman.io/).
   * Use Gradle to build and install Fast-RTPS-Gen.

```
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \
    && cd ~/Fast-RTPS-Gen \
    && gradle assemble \
    && sudo env "PATH=$PATH" gradle install
```

4. Install dependencies for PX4: `PX4-Autopilot/Tools/setup/ubuntu.sh`.
5. Install officially provided ROS2 plug-ins for Gazebo: `sudo apt install ros-foxy-gazebo-ros-pkgs`.
6. Install [QgroundControl](http://qgroundcontrol.com/) ground station.

## Usage

### Simulation with Gazebo

1. Build ROS2 workspace and source this workspace.

```
cd scripts
./build_all_ros_clean.sh
```

2. Build and run PX4 SITL simulation. It is required to source ROS2 setup file.

```
cd PX4-Autopilot
PX4_NO_FOLLOW_MODE=1 px4_sitl_ctrlalloc gazebo_omni_hex
```

3. Run `micrortps_agent` over UDP.

```
micrortps_agent -t UDP
```

4. Use QGC to give commands to the simulation environment. Several useful buttons and modes are listed below.
   - Takeoff and land
   - Virtual joysticks
   - Position mode, altitude mode, ACRO mode
5. Or use the offboard control node to generate a trajectory for omniHex to track.

```
ros2 run px4_ros_com offboard_control
```

### Real World Flight

> TODO: servo driver, VRPN client, real flight

## Software Version

1. The original version of `PX4` is `stable release v1.12.3`. Modified submodules derive from the commit that is referenced by PX4.
2. The original version of `px4_msg` is commit `0f550f436547fccc6b86d448e095ad914f5de94a` on master branch.
3. `px4_ros_com`: commit `c618d757bd64113ccbee17ad0ae17ab8122337e8` on master branch.

## Coordinate Frame

1. SDF and Solidworks models
   - front-x, left-y, up-z (body)
2. PX4
   - NED (world)
   - front-x, right-y, down-z (body)
3. Gazebo
   - N-Green-y, E-Red-x, U-Blue-z (world) 
4. Arm rotation: see [Control Allocation of a Tilting Rotor Hexacopter](https://doi.org/10.3929/ethz-b-000224598)

## SW Model to SDF

1. There are several useful links
   - [Gazebo official guide](https://gazebosim.org/tutorials?cat=build_robot)
   - [Format standard of `.sdf`](http://sdformat.org/spec)
2. Do NOT use SW2018SP0
3. Minimize the number of links. It is good to treat parts that do not move relatively as a whole. Only separate parts into links if there must be a joint in between. Usually, a link corresponds to an assembly.
4. In the assembly that represents a link, make sure to attach a coordinate frame in the SDF convention to the model. This frame will be used as the reference frame when exporting `.stl` and calculating mass properties.
5. When exporting to `.stl` files, hide fine parts otherwise your file would be too big. Choose the aforementioned reference frame, use meter as the scale, and select "do not transform to positive space".
6. Write the SDF
   - SDF uses kilogram and meter as the default unit.
   - Don't use mesh as collision. Use the geometry shapes instead.
   - Specify a `min_depth` property to the collision that is in contact with the ground.
   - Use numbers in "Moments of inertia, taken at the center of mass and aligned with the output coordinate system". Numbers that are not on the diagonal line of the matrix should be inverted.
7. `sitl_gazebo` plug-ins: "IMU" should be loaded after "mavlink interface".

## ROS2 Packages

### custom_gazebo_plugins

1. This package is a supplement to [officially provided ROS2 plug-ins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/foxy) and those provided in `PX4-Autopilot/Tool/sitl_gazebo`. The official repository provides example code for writing a plug-in.
2. `gazebo_ros_arm_rotation.cpp` is for controlling the position of six arm joints of omniHex via the `PID` API. Similarly, `gazebo_ros_joint_pid_ctrl.cpp` controls the position of a single joint via `PID`.
3. `gazebo_ros_joint_motor.cpp` is for controlling joint position or velocity via `SetParam` and `SetPosition` API.
4. `gazebo_ros_motor_model.cpp` is a simplified ROS2 adaptation of the original version in `PX4-Autopilot/Tool/sitl_gazebo`.

### px4_msgs

1. This package contains message types needed by `px4_ros_com` and other packages that might use the PX4 types, as well as `rqt` utility.
2. The `msg` folder is updated by the script in `PX4-Autopilot/msg/tools/uorb_to_ros_msgs.py`. It should be synced whenever a message definition in `PX4-Autopilot/msg` is changed.

### px4_ros_com

1. The main purpose of this package is to generate the communication bridge `micrortps_agent` from a template file `src/templates/uorb_rtps_message_ids.yaml`. The template file should be synced with PX4 via script `PX4-Autopilot/msg/tools/uorb_to_ros_rtps_ids.yaml`.
2. There is also an example of off-board control available in `src/examples/offboard`. This example code is modified to generate a full position-pose trajectory as the function of timestamps.
3. It also provides useful scripts for cleaning and building the workspace.
   * Previously, `build_ros2_workspace.bash` is set to skip `custom_gazebo_plugins` because the latter depend on `px4_msgs`.
   * It is better to add `<depend>px4_msg</depend>` in `package.xml`. In this way we don't need to specify a order. Make sure to add `find_package(px4_msgs REQUIRED)` and `ament_target_dependencies(foo ...px4_msgs...)` in `CMakeLists.txt`.

## PX4 Tools and Miscellaneous

1. Set the correct URL in `.gitmodules`.
2. Custom models for Gazebo simulation is put in `PX4-Autopilot/Tools/sitl_gazebo/models` with a corresponding `.world` file in `PX4-Autopilot/Tools/sitl_gazebo/worlds`.
   - The initial condition of the simulation could be set in `PX4-Autopilot/Tools/sitl_run.sh`.
   - Some plug-ins are built externally (in the ROS2 workspace). So `PX4-Autopilot/Tools/setup_gazebo.bash` is modified to export the correct library paths.
3. As we are using the `micrortps_bridge`, we need to build this module in PX4. We can specify this feature by uncommenting the `micrortps_bridge` line in `PX4-Autopilot/boards/px4/sitl/ctrlalloc.cmake`, which is the module configuration file for SITL targets. We can choose what modules to be built and what not to be built. The same rule applies to `.cmake` files for other boards.
4. Add our custom model for gazebo simulation in `PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake`.
5. Add our airframe file (a model-dependent script for loading parameters and modules) for simulation in `PX4-Autopilot/ROMFS/init.d-posix/airframes` and make sure to add this file to the `CMakeLists.txt`.

## PX4 Messages

1. Add a message to hold arm positions and limit status: `arm_rotation.msg`. After adding a new message file, don't forget to indicate it in `CmakeList.txt`, otherwise it won't be compiled.
2. Every message should be assigned an ID in `msg/tools/urob_rtps_message_ids.yaml` or targets with `micrortps_bridge` enabled cannot be compiled. Also, we can specify how `micrortps_bridge` treat each message.
3. `vehicle_attitude_setpoint.msg` and `vehicle_local_position_setpoint.msg` are modified to handle roll and pitch movements.

## PX4 Modules

- Generally, a good way to read the code is to look at the header files first. Look at topic subscriptions and publications. Then look at the `run()` function in `.cpp` files.
- Modified modules are `angular_velocity_controller`, `commander`, `control_allocator`, `flight_mode_manager`, `mc_att_control` and `mc_pos_control`.

### angular_velocity_controller

1. Center of mass compensation $x_{com} \cross F_{sp}$ in `AngularVelocityControl.cpp/hpp`, `void AngularVelocityControl::update()`.

2. Read center of mass parameters and call `AngularVelocityControl::update()` with additional arguments in`AngularVelocityController.cpp/hpp`.

3. Newton-Euler equation cross term $\omega_b \cross v_b$​ in `void AngularVelocityController::Run()`.

   > TODO: implement Newton-Euler equation correctly

4. Center of mass parameters are defined in `vehicle_model_params.c`.



### commander

1. In `void Commander::update_control_mode()`, ACRO mode is added with the same set of configuration as POSCTL mode.

### control_allocator

1. The allocation matrix (actuator effectiveness) is implemented as a subclass of `ControlAllocationPseudoInverse` and `ModuleParams`. See `ActuatorEffectivenessOmniHex.cpp/hpp/params.c` for more detail. In PX4 convention, torque comes above force in a wrench.
2. Add omniHex to enum classes, switch cases, and includes in `ControlAllocator.cpp/hpp`.
3. Always check `CMakeLists.txt`

### flight_mode_manager

1. This module takes care of controller setpoints in different flight modes.

2. Switch to `FlightTaskIndex::ManualAcceleration` when in `vehicle_status_s::NAVIGATION_STATE_ACRO`. This feature is implemented in `FlightModeManager.cpp`.

   > TODO: exclusive flight task for ACRO mode (not only pitch but full pose)

3. `FlightTask.cpp/hpp`: add missing variables and functions that are not implemented for roll and pitch commands.

4. `FlightTaskManualAltitude.hpp`: add scaling factors from stick to roll and pitch rate.

5. `FlightTaskManualAcceleration.cpp/hpp`: generate different setpoints in different navigation states.

6. Set roll pitch angle and angular speed setpoint in `FlightTaskFoo::activate()` function of tasks such as `Auto`, `Failsafe` and `ManualAltitudeSmoothVel`. This is necessary because otherwise the vehicle loses roll and pitch control when auto buttons in QGC is pressed.

### mc_att_control

1. `AttitudeControl.cpp`: `matrix::Vector3f AttitudeControl::update()` is modified to consider turning rate feed-forward.
2. `AttitudeControl.hpp`: add roll and pitch in `void setAttitudeSetpoint()`, also variables.
3. `mc_att_control_main.cpp` and `mc_att_control.hpp`: call modified function with the right arguments.

### mc_pos_control

1. `PositionControl.cpp/hpp`: add variables to take care of roll and pitch rotation. In `void PositionControl::getAttitudeSetpoint()` attitude setpoints are passed down and thrust setpoint is converted into body frame.
2. `mc_pos_control_params.c`: define parameters for manual roll and pitch rate control, which are used in `FlightTaskManualAltitude.hpp`.
3. `MulticopterPositionControl.cpp/hpp`: call modified functions and pass correct arguments.

## Motor-Propeller Model

### SITL

### Real World Data

> TODO: mapping between u, PWM, angular velocity, determine thrust and torque coefficient