# cubemars_hardware

This package contains a [ros2_control](https://control.ros.org/master/index.html) hardware interface to control the CubeMars AK series motors with a  `SystemInterface`. It was tested on two AK70-10.

## Motor Setup
The package assumes that the motor is setup in Servo Mode.

To adjust the motor settings you can connect the motor with the R-link and use the "Upper Computer" program. All the informations are on the CubeMars [Technical Support and Download page](https://www.cubemars.com/article.php?id=261).

Make sure that you enable "Send status over CAN" (called `send_can_status` in AppParams). Otherwise the state interfaces will not work proberly. Also, the upload frequency of the actuators should be at least as fast as the update rate of the controller manager. If no CAN message is received during one update loop you will get a warning in the `read` function.

Example motor settings can be found in [our main repo](https://github.com/OpenFieldAutomation-OFA/ros-weed-control/tree/main/.motor_params/cubemars).

## SocketCAN
SocketCAN is used to handle the communication between the motors and the Linux system. You need a working CAN interface for this to work. The exact setup will depend on your hardware but it will include initializing the interface.
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```
When typing `ifconfig` you should now see the CAN interface. You can also read all incoming messages.
```bash
sudo apt install can-utils
candump can0
```

## Hardware Interfaces
The following command interfaces are published:
- `position`: Position(-Speed) Loop Mode
- `velocity`: Speed Loop Mode
- `effort`: Current Loop Mode

Your controller can only claim one of the command interfaces. Claiming multiple command interfaces at the same time is not possible.

The following state interfaces are published:
- `position`
- `velocity`
- `effort`
- `temperature`

The hardware interfaces can also be listed by starting the controller manager and running the following command.
```
ros2 control list_hardware_interfaces
```

## `ros2_control` Parameters
An example `ros2_control` URDF config with this hardware interface can be found in [our main repo](https://github.com/OpenFieldAutomation-OFA/ros-weed-control/blob/main/ofa_moveit_config/ros2_control/ofa_robot.ros2_control.xacro).

`hardware` tag:
- `can_interface`: name of the Linux CAN interface, e. g. `can0`

`joint` tag:
- `can_id`: CAN ID of the actuator
- `pole_pairs`: Pole pairs. Used for unit conversion
- `gear_ratio`: Gear ratio. Used for unit conversion
- `kt`: Torque constant. Used to convert current to torque
- `trq_limit`: OPTIONAL. Torque limit. If the limit is reached the motor is disabled and the hardware interface is shutdown.
- `enc_off`: OPTIONAL. Encoder offset in rad. (see explanation below)
- `vel_limit`: OPTIONAL. Velocity limit in rad/s. (see explanation below)
- `acc_limit`: OPTIONAL. Acceleration limit in rad/s^2. (see explanation below)
- `read_only`: OPTIONAL. If set to 1, the current position is logged and no commands are sent to the motors.

## Encoder Offset
For single-encoder motors there will most likely be an offset between your desired origin and the zero of the encoder on startup. You can compensate for this offset by using `enc_off`. Note that in order for `enc_off` to work properly, the motor has to be very close to the origin on startup, otherwise the encoder value will wrap around.

For motors with an output encoder you should be able to manually set the origin just once using the following command (replace `XX` with the hexadecimal representation of the CAN ID).
```
cansend can0 000005XX#01
```

## Velocity and Acceleration Limits
The `position` command interface will by default use the Position Mode (control mode 4) where the motor runs to the specified position at maximum speed and acceleration. If you want to use the Position-Speed Loop Mode (control mode 6) you have to specify BOTH `vel_limit` and `acc_limit`. This will limit the maximum acceleration and velocity of the motor (trajectory planning). This does not work well together with a `joint_trajectory_controller`.

Note that the limit parameters have no effect if only one is set or if you don't use the `position` command interface.

