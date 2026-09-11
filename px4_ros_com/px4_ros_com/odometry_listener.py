import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

from px4_msgs.msg import VehicleOdometry


class OdometryListener(Node):

    def __init__(self):
        super().__init__('odometry_listener')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )

        self.get_logger().info(
            'PX4 Odometry Listener started.'
        )

    def odometry_callback(self, msg):

        x = msg.position[0]
        y = msg.position[1]
        z = msg.position[2]

        self.get_logger().info(
            f'Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = OdometryListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
