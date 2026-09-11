import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# 三台各自出生點在「共同世界座標」下的位置 (north, east)，單位公尺。
# 這組數字是從實際 GPS (vehicle_global_position) 反推驗證過的，
# 對應啟動時給的 PX4_GZ_MODEL_POSE="0,0" / "0,2" / "0,-2"。
# 每台 PX4 自己的 local frame 原點都是「自己的出生點」，
# 所以要換算成共同座標，才能正確算出「主機前方 5m」實際對到哪台從機的哪個 local 座標。
SPAWN_NED = {
    'px4_1': {'north': 0.0, 'east': 0.0},
    'px4_2': {'north': 2.0, 'east': 0.0},
    'px4_3': {'north': -2.0, 'east': 0.0},
}

LEADER_NS = 'px4_1'
LEADER_TARGET_SYSTEM = 2
LEADER_HOVER_ALTITUDE = 3.0  # 主機自己起飛的高度 (m)

FORWARD_OFFSET_M = 5.0  # 往前 = 世界座標固定的北方，不隨主機機頭轉動
UP_OFFSET_M = 5.0       # 從機比主機高多少 (m)

FOLLOWERS = [
    # lateral_offset_east: 額外左右間距，避免兩台從機疊在同一點相撞
    {'namespace': 'px4_2', 'target_system': 3, 'lateral_offset_east': 1.5},
    {'namespace': 'px4_3', 'target_system': 4, 'lateral_offset_east': -1.5},
]


class LeaderTakeoff(Node):
    """主機：起飛到固定高度並懸停，行為跟 sync_takeoff.py 的 DroneTakeoff 相同。"""

    def __init__(self, namespace: str, target_system: int, hover_altitude: float):
        super().__init__(f'{namespace}_leader')
        self.target_system = target_system
        self.hover_altitude = hover_altitude

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'/{namespace}/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'/{namespace}/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, f'/{namespace}/fmu/in/vehicle_command', PX4_QOS)

        self.setpoint_counter = 0
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # 剛開機的 SITL 需要幾秒讓 EKF/GPS home 穩定下來，太早送 ARM 會被
        # preflight check 拒絕、而且這裡沒有重試，所以要等夠久（5 秒）再送一次。
        if self.setpoint_counter == 50:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info('[主機] offboard + arm 指令已送出')

        if self.setpoint_counter < 51:
            self.setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -self.hover_altitude]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1, msg.param2, msg.command = param1, param2, command
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)


class FollowerFormation(Node):
    """從機：即時追蹤主機的 local position，換算成共同世界座標後，
    加上『往前 5m、往上 5m、左右間距』的固定 offset，換算回自己的 local frame 送出去。
    """

    def __init__(self, namespace: str, target_system: int, lateral_offset_east: float):
        super().__init__(f'{namespace}_follower')
        self.target_system = target_system
        self.lateral_offset_east = lateral_offset_east
        self.own_spawn = SPAWN_NED[namespace]
        self.leader_spawn = SPAWN_NED[LEADER_NS]

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'/{namespace}/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'/{namespace}/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, f'/{namespace}/fmu/in/vehicle_command', PX4_QOS)

        self.leader_x = 0.0
        self.leader_y = 0.0
        self.leader_z = 0.0
        self.leader_position_valid = False

        self.leader_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f'/{LEADER_NS}/fmu/out/vehicle_local_position_v1',
            self.leader_position_callback,
            PX4_QOS,
        )

        self.setpoint_counter = 0
        self.create_timer(0.1, self.timer_callback)

    def leader_position_callback(self, msg: VehicleLocalPosition):
        self.leader_x = msg.x
        self.leader_y = msg.y
        self.leader_z = msg.z
        self.leader_position_valid = True

    def compute_target(self):
        # 1. 主機 local (x,y) -> 共同世界座標 (north, east)
        leader_world_north = self.leader_x + self.leader_spawn['north']
        leader_world_east = self.leader_y + self.leader_spawn['east']

        # 2. 加上「往前 5m」跟「左右間距」，得到這台從機的世界座標目標點
        target_world_north = leader_world_north + FORWARD_OFFSET_M
        target_world_east = leader_world_east + self.lateral_offset_east

        # 3. 世界座標 -> 換算回這台從機自己的 local frame
        target_x = target_world_north - self.own_spawn['north']
        target_y = target_world_east - self.own_spawn['east']
        target_z = self.leader_z - UP_OFFSET_M  # z 越負代表越高，所以是「減」

        return target_x, target_y, target_z

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # 理由同主機：剛開機需要幾秒讓 EKF/GPS home 穩定，這裡沒有重試機制，
        # 等夠久（5 秒）再送一次 ARM。
        if self.setpoint_counter == 50:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info('[從機] offboard + arm 指令已送出')

        if self.setpoint_counter < 51:
            self.setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        if self.leader_position_valid:
            x, y, z = self.compute_target()
        else:
            # 還沒收到主機位置前，先在自己出生點正上方 1m 低調懸停，
            # 但仍要維持 setpoint stream，不然 PX4 永遠進不了 offboard。
            x, y, z = 0.0, 0.0, -1.0

        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1, msg.param2, msg.command = param1, param2, command
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    nodes = [LeaderTakeoff(LEADER_NS, LEADER_TARGET_SYSTEM, LEADER_HOVER_ALTITUDE)]
    nodes += [
        FollowerFormation(f['namespace'], f['target_system'], f['lateral_offset_east'])
        for f in FOLLOWERS
    ]

    executor = MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)

    try:
        executor.spin()
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
