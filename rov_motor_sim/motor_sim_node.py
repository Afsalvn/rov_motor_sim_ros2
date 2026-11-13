"""Motor simulator node: models three thrusters and publishes their RPM."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

from rov_motor_sim.kinematics import clamp

# order of the values in every Float32MultiArray in this package
MOTOR_NAMES = ('left', 'right', 'vertical')

WATCHDOG_WARN_PERIOD_S = 5.0    # so the warning doesn't flood the log


class MotorSimNode(Node):
    """Stands in for the motor controller until there is real hardware."""

    def __init__(self):
        super().__init__('motor_sim_node')

        self.tau = self.declare_parameter('tau', 0.2).value
        self.max_accel_rpm_per_s = self.declare_parameter(
            'max_accel_rpm_per_s', 0.0).value
        self.sim_rate_hz = self.declare_parameter('sim_rate_hz', 50.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value

        self.dt = 1.0 / self.sim_rate_hz

        self.target_rpm = [0.0, 0.0, 0.0]
        self.current_rpm = [0.0, 0.0, 0.0]
        self.last_cmd_time = self.get_clock().now()

        self.rpm_cmd_sub = self.create_subscription(
            Float32MultiArray, 'motor_rpm_cmd', self.on_rpm_cmd, 10)
        self.rpm_pub = self.create_publisher(
            Float32MultiArray, 'motor_rpm', 10)

        # also one Float32 topic per motor (motor_rpm/left etc). Easier to
        # plot one motor than to pick an index out of the array.
        self.motor_pubs = []
        for name in MOTOR_NAMES:
            self.motor_pubs.append(
                self.create_publisher(Float32, 'motor_rpm/' + name, 10))

        # Step the sim from a timer and not from the subscriber callback,
        # otherwise dt depends on how fast commands come in and telemetry
        # stops when they stop.
        self.timer = self.create_timer(self.dt, self.on_timer)

    def on_rpm_cmd(self, msg):
        # just latch the targets here, on_timer does the actual stepping
        self.target_rpm = list(msg.data)
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        self.check_watchdog()

        for i in range(len(MOTOR_NAMES)):
            self.current_rpm[i] = self.step_motor(
                self.current_rpm[i], self.target_rpm[i])

        self.publish_rpm()

    def check_watchdog(self):
        """Zero the targets if commands have stopped arriving."""
        elapsed = self.get_clock().now() - self.last_cmd_time

        if elapsed.nanoseconds / 1e9 > self.cmd_timeout:
            # Only the targets are zeroed, so the motors still spin down
            # through the lag instead of jumping to 0.
            self.target_rpm = [0.0, 0.0, 0.0]
            self.get_logger().warn(
                f'No RPM command for more than {self.cmd_timeout} s, '
                'targets set to zero',
                throttle_duration_sec=WATCHDOG_WARN_PERIOD_S)

    def step_motor(self, rpm, target):
        """Return the RPM of one motor after one time step."""
        # first-order lag, forward Euler
        change = (target - rpm) * self.dt / self.tau

        # optional slew limit, 0 (or negative) = off
        if self.max_accel_rpm_per_s > 0.0:
            max_change = self.max_accel_rpm_per_s * self.dt
            change = clamp(change, max_change)

        return rpm + change

    def publish_rpm(self):
        rpm_array = Float32MultiArray()
        rpm_array.data = self.current_rpm
        self.rpm_pub.publish(rpm_array)

        for publisher, rpm in zip(self.motor_pubs, self.current_rpm):
            rpm_single = Float32()
            rpm_single.data = rpm
            publisher.publish(rpm_single)


def main(args=None):
    rclpy.init(args=args)
    node = MotorSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass    # Ctrl-C or launch shutting us down
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
