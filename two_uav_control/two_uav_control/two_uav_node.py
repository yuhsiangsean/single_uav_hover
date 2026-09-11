import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand


class TwoUAVControl(Node):

    def __init__(self):
        super().__init__('two_uav_control')

        # =========================================================
        # PX4 input QoS
        #
        # We already checked PX4:
        # Reliability: BEST_EFFORT
        # Durability: VOLATILE
        # =========================================================
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # =========================================================
        # UAV 1
        # Namespace: /px4_1
        # PX4 SYS_ID: 2
        # =========================================================

        self.uav1_offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/px4_1/fmu/in/offboard_control_mode',
            px4_qos
        )

        self.uav1_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/px4_1/fmu/in/trajectory_setpoint',
            px4_qos
        )

        self.uav1_command_pub = self.create_publisher(
            VehicleCommand,
            '/px4_1/fmu/in/vehicle_command',
            px4_qos
        )

        # =========================================================
        # UAV 2
        # Namespace: /px4_2
        # PX4 SYS_ID: 3
        # =========================================================

        self.uav2_offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/px4_2/fmu/in/offboard_control_mode',
            px4_qos
        )

        self.uav2_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/px4_2/fmu/in/trajectory_setpoint',
            px4_qos
        )

        self.uav2_command_pub = self.create_publisher(
            VehicleCommand,
            '/px4_2/fmu/in/vehicle_command',
            px4_qos
        )

        # 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.counter = 0
        self.command_sent = False

        self.get_logger().info(
            'Two UAV Control Node started!'
        )

        self.get_logger().info(
            'UAV 1: /px4_1, SYS_ID=2'
        )

        self.get_logger().info(
            'UAV 2: /px4_2, SYS_ID=3'
        )

    def timer_callback(self):

        # =========================================================
        # 1. Continuously publish OffboardControlMode
        # =========================================================

        self.publish_offboard_mode(
            self.uav1_offboard_pub
        )

        self.publish_offboard_mode(
            self.uav2_offboard_pub
        )

        # =========================================================
        # 2. Continuously publish TrajectorySetpoint
        #
        # NED coordinate:
        # x = 0
        # y = 0
        # z = -2
        #
        # approximately 2 meters above origin
        # =========================================================

        self.publish_setpoint(
            self.uav1_setpoint_pub
        )

        self.publish_setpoint(
            self.uav2_setpoint_pub
        )

        self.counter += 1

        # =========================================================
        # 3. After 2 seconds:
        #    OFFBOARD + ARM both UAVs
        # =========================================================

        if self.counter == 20:

            self.get_logger().info(
                'Sending OFFBOARD + ARM commands to BOTH UAVs'
            )

            # -----------------------------------------------------
            # UAV 1
            # PX4 SYS_ID = 2
            # -----------------------------------------------------

            self.publish_vehicle_command(
                self.uav1_command_pub,
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
                2
            )

            self.publish_vehicle_command(
                self.uav1_command_pub,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
                0.0,
                2
            )

            # -----------------------------------------------------
            # UAV 2
            # PX4 SYS_ID = 3
            # -----------------------------------------------------

            self.publish_vehicle_command(
                self.uav2_command_pub,
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
                3
            )

            self.publish_vehicle_command(
                self.uav2_command_pub,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
                0.0,
                3
            )

            self.command_sent = True

    def publish_offboard_mode(self, publisher):

        msg = OffboardControlMode()

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        publisher.publish(msg)

    def publish_setpoint(self, publisher):

        msg = TrajectorySetpoint()

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        # PX4 NED coordinate system
        #
        # z = -2.0
        # means approximately 2 meters above the origin

        msg.position = [
            0.0,
            0.0,
            -2.0
        ]

        msg.yaw = 0.0

        publisher.publish(msg)

    def publish_vehicle_command(
        self,
        publisher,
        command,
        param1=0.0,
        param2=0.0,
        target_system=1
    ):

        msg = VehicleCommand()

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        msg.param1 = param1
        msg.param2 = param2

        msg.command = command

        # IMPORTANT:
        # UAV 1 = SYS_ID 2
        # UAV 2 = SYS_ID 3

        msg.target_system = target_system
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = TwoUAVControl()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()