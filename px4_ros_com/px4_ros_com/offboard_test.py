import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint
)


class OffboardTest(Node):

    def __init__(self):
        super().__init__('offboard_test')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'Offboard test started.'
        )

    def timer_callback(self):

        # ---------------------------------
        # 1. OffboardControlMode
        # ---------------------------------

        offboard_msg = OffboardControlMode()

        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = self.get_clock().now().nanoseconds // 1000

        self.offboard_control_mode_pub.publish(
            offboard_msg
        )

        # ---------------------------------
        # 2. TrajectorySetpoint
        # ---------------------------------

        trajectory_msg = TrajectorySetpoint()

        trajectory_msg.position[0] = 0.0
        trajectory_msg.position[1] = 0.0
        trajectory_msg.position[2] = -2.0

        trajectory_msg.yaw = 0.0

        trajectory_msg.timestamp = self.get_clock().now().nanoseconds // 1000

        self.trajectory_setpoint_pub.publish(
            trajectory_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = OffboardTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
