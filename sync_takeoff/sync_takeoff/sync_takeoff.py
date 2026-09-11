import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# PX4 SITL 每個 instance 的 MAVLink system_id 是 instance 編號 + 1
# (實測驗證: instance 1 -> system_id 2, instance 2 -> 3, instance 3 -> 4)
DRONES = [
    {'namespace': 'px4_1', 'target_system': 2},
    {'namespace': 'px4_2', 'target_system': 3},
    {'namespace': 'px4_3', 'target_system': 4},
]

TAKEOFF_ALTITUDE = 5.0  # 公尺


class DroneTakeoff(Node):

    def __init__(self, namespace: str, target_system: int, takeoff_altitude: float):
        super().__init__(f'{namespace}_takeoff')
        self.target_system = target_system
        self.takeoff_altitude = takeoff_altitude

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'/{namespace}/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'/{namespace}/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, f'/{namespace}/fmu/in/vehicle_command', PX4_QOS)

        self.setpoint_counter = 0
        self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        # offboard 模式要求持續收到 setpoint stream，否則 PX4 會逾時退出 offboard
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # 送滿 1 秒的 setpoint 之後才切 offboard + arm，
        # 這是 PX4 官方 offboard 範例的標準作法（避免因為 stream 還沒建立就切模式被拒絕）
        if self.setpoint_counter == 10:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)  # OFFBOARD
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)  # ARM
            self.get_logger().info('offboard + arm 指令已送出')

        if self.setpoint_counter < 11:
            self.setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -self.takeoff_altitude]  # NED: z 為負代表往上
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    drones = [
        DroneTakeoff(d['namespace'], d['target_system'], TAKEOFF_ALTITUDE)
        for d in DRONES
    ]

    executor = MultiThreadedExecutor()
    for drone in drones:
        executor.add_node(drone)

    try:
        executor.spin()
    finally:
        for drone in drones:
            drone.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
