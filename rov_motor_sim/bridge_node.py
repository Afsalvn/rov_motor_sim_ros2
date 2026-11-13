"""Bridge node: turns cmd_vel velocity commands into target motor RPMs."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from rov_motor_sim.kinematics import twist_to_rpm


class BridgeNode(Node):
    """Subscribe to cmd_vel and publish [left, right, vertical] RPM targets."""

    def __init__(self):
        super().__init__('bridge_node')

        # defaults here are the same as config/params.yaml
        self.rpm_gain = self.declare_parameter('rpm_gain', 3000.0).value
        self.max_rpm = self.declare_parameter('max_rpm', 3000.0).value
        self.thruster_separation = self.declare_parameter(
            'thruster_separation', 0.3).value

        # relative topic names so the nodes can be namespaced / remapped
        self.rpm_cmd_pub = self.create_publisher(
            Float32MultiArray, 'motor_rpm_cmd', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.on_cmd_vel, 10)

    def on_cmd_vel(self, msg):
        # linear.y, roll and pitch are ignored, can't do those with
        # three thrusters anyway
        left, right, vertical = twist_to_rpm(
            msg.linear.x, msg.angular.z, msg.linear.z,
            self.thruster_separation, self.rpm_gain, self.max_rpm)

        rpm_cmd = Float32MultiArray()
        rpm_cmd.data = [left, right, vertical]    # motor_sim_node expects this order
        self.rpm_cmd_pub.publish(rpm_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
