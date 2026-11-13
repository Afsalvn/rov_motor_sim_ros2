# rov_motor_sim

## Overview

A ROS 2 Python package for an underwater ROV (remotely operated vehicle).
It subscribes to velocity commands on `/cmd_vel`, converts them into motor
RPM values, simulates how the motors respond, and publishes the RPM
telemetry back to ROS 2 topics.

The package has two nodes:

- `bridge_node` listens to `/cmd_vel`, works out the RPM each thruster
  should run at, and publishes those targets.
- `motor_sim_node` stands in for the motor controller. It receives the
  targets, simulates the motors at a fixed rate, and publishes the RPM the
  motors have reached.

All tunable values are ROS parameters in `config/params.yaml`.

## Architecture

```
  teleop_twist_keyboard
  or ros2 topic pub
          |
          |  /cmd_vel              geometry_msgs/Twist
          v
  +----------------+
  |  bridge_node   |   velocity -> RPM (kinematics.py),
  +----------------+   clamped to +/- max_rpm
          |
          |  /motor_rpm_cmd        std_msgs/Float32MultiArray
          |                        [left, right, vertical]
          v
  +----------------+
  | motor_sim_node |   50 Hz timer: first-order lag,
  +----------------+   optional acceleration limit, command watchdog
          |
          |  /motor_rpm            std_msgs/Float32MultiArray
          |                        [left, right, vertical]
          |  /motor_rpm/left       std_msgs/Float32
          |  /motor_rpm/right      std_msgs/Float32
          |  /motor_rpm/vertical   std_msgs/Float32
          v
  ros2 topic echo
```

## Equations

### Velocity to RPM (bridge_node, kinematics.py)

```
left     = linear.x - angular.z * L / 2
right    = linear.x + angular.z * L / 2
vertical = linear.z

rpm = k * v, clamped to the range -max_rpm .. +max_rpm
```

`L` is `thruster_separation` and `k` is `rpm_gain`.

Example with the default values (`k` = 3000, `L` = 0.3 m): a command of
`linear.x = 0.5`, `angular.z = 1.0` gives

```
left  = (0.5 - 1.0 * 0.15) * 3000 = 1050 RPM
right = (0.5 + 1.0 * 0.15) * 3000 = 1950 RPM
```

### Motor response (motor_sim_node)

Every `dt = 1 / sim_rate_hz` seconds, for each motor:

```
change = (target - rpm) * dt / tau                  first-order lag

if max_accel_rpm_per_s > 0:                         optional limit
    change is limited to +/- max_accel_rpm_per_s * dt

rpm = rpm + change
```

### Watchdog (motor_sim_node)

```
if no command has arrived for more than cmd_timeout seconds:
    target = 0 for every motor
```

A throttled warning is logged while this lasts.

## Assumptions

- **Thruster layout.** Three thrusters. A left and a right horizontal
  thruster, a distance `L` apart, give forward motion and yaw. One vertical
  thruster gives up and down motion (heave). The other parts of the Twist
  (`linear.y`, `angular.x`, `angular.y`) are not used.
- **Linear gain.** RPM is proportional to velocity, `rpm = k * v`, with the
  same `k` for all three thrusters.
- **First-order motor model.** Each motor follows its target as a
  first-order lag with one time constant `tau`, the same for all motors.
- **Float32MultiArray instead of a custom message.** A custom message needs
  a separate interface package. Standard messages keep this to one Python
  package. Both arrays always hold three values in the order
  `[left, right, vertical]`.

## Build and run

The package folder must be inside the `src` folder of a colcon workspace.

```bash
source /opt/ros/jazzy/setup.bash        # or humble
cd ~/rov_motor_sim_ws
colcon build
source install/setup.bash
ros2 launch rov_motor_sim motor_sim.launch.py
```

## Testing by hand

Open a new terminal for each command and run
`source ~/rov_motor_sim_ws/install/setup.bash` in it first.

### With teleop_twist_keyboard

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys: `i` forward, `,` backward, `j` and `l` turn left and right, `t` up,
`b` down, `k` stop.

teleop_twist_keyboard sends one message per key press, so hold the key down
to keep commands flowing. When the commands stop, the watchdog stops the
motors after `cmd_timeout`.

### With ros2 topic pub and echo

Send a command 10 times per second (forward 0.5 m/s while turning left):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 1.0}}"
```

Look at the result:

```bash
ros2 topic echo /motor_rpm_cmd     # targets: 1050, 1950, 0
ros2 topic echo /motor_rpm         # simulated RPM rising towards the targets
ros2 topic echo /motor_rpm/left    # one motor on its own
```

## Running the tests

With colcon:

```bash
cd ~/rov_motor_sim_ws
colcon test
colcon test-result --verbose
```

Without ROS (only Python and pytest are needed):

```bash
cd ~/rov_motor_sim_ws/src/rov_motor_sim
python3 -m pytest test/ -v
```

## How this would map to real firmware

On the real ROV, `motor_sim_node` would be replaced by a microcontroller
(for example an STM32) connected over UART.

- `bridge_node` keeps the same conversion, but instead of publishing
  `/motor_rpm_cmd` it sends framed RPM packets over the serial port, for
  example: start byte, length, three RPM values, checksum.
- The STM32 reads the packets, drives the motors, and sends the measured
  RPM back in the same kind of packet.
- The bridge publishes those measured values on `/motor_rpm`, so everything
  else in ROS stays the same.
- The command watchdog would run on the STM32 itself, so the motors stop
  even if the cable or the computer fails.
