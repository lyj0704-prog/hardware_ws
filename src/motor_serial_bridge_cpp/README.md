# motor_serial_bridge_cpp

ROS2 Humble C++ (`rclcpp`) UART bridge for F1TENTH Jetson <-> ESP32 motor control.

## Safety / Wiring Notes

- UART direct wiring assumes **3.3V logic level** on both Jetson and ESP32.
- Connect TX<->RX crossed and GND shared.
- Select the board-specific serial device by parameter: `/dev/ttyTHS1`, `/dev/ttyAMA0`, or `/dev/ttyUSB0`.

## Features

- Subscribe `/ackermann_cmd` (`ackermann_msgs/msg/AckermannDriveStamped`)
- Optional subscribe `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Emergency stop latch via `/emergency_stop` (`std_msgs/msg/Bool`)
- Periodic UART command output with timeout-safe stop
- UART feedback parser for `FB ...` lines (partial frame tolerant)
- Publish `/odom`, TF `odom->base_link`, optional `/joint_states`, `/wheel_ticks`, `/diagnostics`
- Serial reconnect without crashing

## UART Examples

- TX (Jetson->ESP32):
  - `CMD spd=0.80 steer=0.12 estop=0`
  - `CMD pwm=1200 steer_us=1500 estop=0`
- RX (ESP32->Jetson):
  - `FB t=123456 enc=98765 vel=12.3 batt=7.9 fault=0`
  - `FB t=123456 encL=123 encR=124 vel=10.0 fault=0`

## Build

```bash
cd /home/seok/perception_ws
colcon build --packages-select motor_serial_bridge_cpp
source install/setup.bash
```

## Run

```bash
ros2 launch motor_serial_bridge_cpp motor_serial_bridge.launch.py
```

Or with a custom config:

```bash
ros2 launch motor_serial_bridge_cpp motor_serial_bridge.launch.py config:=/path/to/motor_serial_bridge.yaml
```

## Joy To Ackermann (Manual Teleop)

This package also provides `joy_to_ackermann_node` for manual control mapping:

- Subscribe `/joy_left` (`sensor_msgs/msg/Joy`) for steering
- Subscribe `/joy_right` (`sensor_msgs/msg/Joy`) for throttle buttons
- Publish `/ackermann_cmd` (`ackermann_msgs/msg/AckermannDriveStamped`)
- Typical use: left stick steering + A button forward

Run:

```bash
ros2 launch motor_serial_bridge_cpp joy_to_ackermann.launch.py
```

Example to run split Joy-Con inputs:

```bash
ros2 run joy joy_node --ros-args -p device_id:=0 -r /joy:=/joy_left
ros2 run joy joy_node --ros-args -p device_id:=1 -r /joy:=/joy_right
```

With custom mapping config:

```bash
ros2 launch motor_serial_bridge_cpp joy_to_ackermann.launch.py config:=/path/to/joy_to_ackermann.yaml
```

## Gamepad To Ackermann

For gamepad manual control:

- `axes[0]`: steering (left/right)
- `buttons[1]`: forward hold (accelerate up to max forward speed)
- `buttons[3]`: reverse hold (accelerate up to max reverse speed)
- `steering_slew_radps`: steering angle change limit (rad/s). Lower value = slower steering response.
- `steering_threshold`: abs(axis) threshold for steering activation (`0.0` to `1.0`).
  Above threshold, steering target snaps to `+/-max_steer_rad`, and actual steering follows at `steering_slew_radps`.

Run:

```bash
ros2 launch motor_serial_bridge_cpp gamepad_to_ackermann.launch.py
```

Custom config:

```bash
ros2 launch motor_serial_bridge_cpp gamepad_to_ackermann.launch.py config:=/path/to/gamepad_to_ackermann.yaml
```
