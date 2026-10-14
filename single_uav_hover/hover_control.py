from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)

import rclpy

from rclpy.node import Node

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy
)

from rclpy.signals import SignalHandlerOptions

from std_msgs.msg import Int32


class SingleUAVHover(Node):

    def __init__(self):

        super().__init__('single_uav_hover_control')

        # ============================================================
        # Parameters
        #
        # namespace: PX4 uXRCE-DDS topic prefix. Defaults to
        # "/MAV4", matching the real hardware setup this vehicle
        # flies with (mocap_px4_bridge on mav1-leader publishes mocap
        # odometry to /MAV4/fmu/in/vehicle_visual_odometry, so PX4's
        # uXRCE-DDS client on this vehicle is namespaced /MAV4). For
        # SITL or a different vehicle, override via
        # --ros-args -p namespace:=<value> to match whatever
        # namespace the flight controller's uXRCE-DDS client is
        # actually configured with.
        # ============================================================

        self.declare_parameter('namespace', '/MAV4')
        self.declare_parameter('hover_altitude_m', 0.5)
        self.declare_parameter('climb_rate_mps', 0.3)
        self.declare_parameter('land_descent_rate_mps', 0.3)
        self.declare_parameter('horizontal_speed_mps', 0.2)
        self.declare_parameter('horizontal_step_m', 0.5)

        self.namespace = self.get_parameter(
            'namespace'
        ).get_parameter_value().string_value

        hover_altitude_m = self.get_parameter(
            'hover_altitude_m'
        ).get_parameter_value().double_value

        self.climb_rate = self.get_parameter(
            'climb_rate_mps'
        ).get_parameter_value().double_value

        self.land_descent_rate = self.get_parameter(
            'land_descent_rate_mps'
        ).get_parameter_value().double_value

        self.horizontal_speed = self.get_parameter(
            'horizontal_speed_mps'
        ).get_parameter_value().double_value

        self.horizontal_step = self.get_parameter(
            'horizontal_step_m'
        ).get_parameter_value().double_value

        # NED z is negative up; hover_altitude_m is given as a
        # positive height above the arming point.
        self.hover_z = -abs(hover_altitude_m)

        self.get_logger().info(
            'Single UAV Hover Node started! '
            f'namespace={self.namespace}, '
            f'hover_altitude={hover_altitude_m:.2f}m'
        )

        # ============================================================
        # PX4 QoS (matches PX4's uXRCE-DDS side: BEST_EFFORT +
        # TRANSIENT_LOCAL, KEEP_LAST depth 1 - a mismatched profile
        # means subscriptions silently receive nothing)
        # ============================================================

        self.px4_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ============================================================
        # Publishers
        # ============================================================

        in_prefix = f'{self.namespace}/fmu/in'

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'{in_prefix}/offboard_control_mode',
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{in_prefix}/trajectory_setpoint',
            10
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            f'{in_prefix}/vehicle_command',
            10
        )

        # ============================================================
        # Subscribers
        # ============================================================

        out_prefix = f'{self.namespace}/fmu/out'

        self.position = None
        self.status = None

        self.create_subscription(
            VehicleLocalPosition,
            f'{out_prefix}/vehicle_local_position_v1',
            self.position_callback,
            self.px4_qos
        )

        self.create_subscription(
            VehicleStatus,
            f'{out_prefix}/vehicle_status_v1',
            self.status_callback,
            self.px4_qos
        )

        # Command from the separate hover_commander terminal (see
        # command_callback()): 1 = takeoff to hover, 2 = land,
        # 3 = +X step, 4 = +Y step. Kept in its own process so the
        # operator's input prompt doesn't get scrolled away by this
        # node's own status logging.

        self.create_subscription(
            Int32,
            '/single_uav_hover/command',
            self.command_callback,
            10
        )

        # ============================================================
        # State machine
        #
        # idle       - on the ground, disarmed, waiting for command 1
        # taking_off - OFFBOARD + ARM requested, ramping up to hover_z
        # hovering   - holding hover_z, waiting for command 2
        # landing    - ramping target_z back down to the ground,
        #              disarms once close to it and returns to idle
        # ============================================================

        self.state = 'idle'

        self.offboard_requested = False
        self.arming_requested = False

        # Setpoint currently being published. Starts at the ground
        # (0.0) so the setpoint stream PX4 requires before it will
        # accept an OFFBOARD switch is already flowing from node
        # startup, well before command 1 is ever sent.
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        # Horizontal targets (goal_x/goal_y) that target_x/target_y
        # ramp toward at horizontal_speed while hovering - commands
        # 3/4 add horizontal_step to these, they never jump directly.
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.dt = 0.1

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        self.last_status_print = 0.0

    # ================================================================
    # Callbacks
    # ================================================================

    def position_callback(self, msg):

        self.position = msg

    def status_callback(self, msg):

        self.status = msg

    # ================================================================
    # Timestamp
    # ================================================================

    def timestamp(self):

        return self.get_clock().now().nanoseconds // 1000

    # ================================================================
    # OffboardControlMode
    # ================================================================

    def publish_offboard_control(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(msg)

    # ================================================================
    # Trajectory setpoint (always straight up/down over the arming
    # point - this package only ever hovers in place)
    # ================================================================

    def publish_setpoint(self, x, y, z):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        msg.position = [float(x), float(y), float(z)]

        msg.yaw = 0.0

        self.setpoint_pub.publish(msg)

    # ================================================================
    # Vehicle command
    # ================================================================

    def send_command(self, command, param1=0.0, param2=0.0):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = command

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        # target_system=0 is treated by PX4 as a broadcast address;
        # this node only ever talks to the one vehicle on its own
        # private namespace's vehicle_command topic.
        msg.target_system = 0
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    # ================================================================
    # Status checks
    # ================================================================

    def is_offboard(self):

        if self.status is None:
            return False

        return (
            self.status.nav_state
            ==
            VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def is_armed(self):

        if self.status is None:
            return False

        return (
            self.status.arming_state
            ==
            VehicleStatus.ARMING_STATE_ARMED
        )

    def request_offboard(self):

        self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

        self.get_logger().info('OFFBOARD command sent')

        self.offboard_requested = True

    def request_arm(self):

        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

        self.get_logger().info('ARM command sent')

        self.arming_requested = True

    # ================================================================
    # Command from the separate hover_commander terminal
    # ================================================================

    def command_callback(self, msg):

        command = msg.data

        if command == 1:

            if self.state == 'idle':

                self.state = 'taking_off'

                self.offboard_requested = False
                self.arming_requested = False

                self.get_logger().info(
                    'Command 1 received: taking off to '
                    f'{-self.hover_z:.2f}m hover'
                )

            else:

                self.get_logger().info(
                    f'Command 1 ignored: already in state '
                    f'"{self.state}"'
                )

        elif command == 2:

            if self.state in ('taking_off', 'hovering'):

                self.state = 'landing'

                self.get_logger().info(
                    'Command 2 received: landing'
                )

            else:

                self.get_logger().info(
                    f'Command 2 ignored: not flying '
                    f'(state="{self.state}")'
                )

        elif command == 3:

            if self.state == 'hovering':

                self.goal_x += self.horizontal_step

                self.get_logger().info(
                    'Command 3 received: moving to '
                    f'x={self.goal_x:.2f}m'
                )

            else:

                self.get_logger().info(
                    f'Command 3 ignored: not hovering '
                    f'(state="{self.state}")'
                )

        elif command == 4:

            if self.state == 'hovering':

                self.goal_y += self.horizontal_step

                self.get_logger().info(
                    'Command 4 received: moving to '
                    f'y={self.goal_y:.2f}m'
                )

            else:

                self.get_logger().info(
                    f'Command 4 ignored: not hovering '
                    f'(state="{self.state}")'
                )

        else:

            self.get_logger().info(
                f'Unknown command "{command}", use 1 (hover), '
                '2 (land), 3 (+X 0.5m) or 4 (+Y 0.5m)'
            )

    # ================================================================
    # Print status
    # ================================================================

    def print_status(self):

        if self.position is not None:

            self.get_logger().info(
                f'state={self.state}, '
                f'x={self.position.x:.2f} '
                f'(target={self.target_x:.2f}), '
                f'y={self.position.y:.2f} '
                f'(target={self.target_y:.2f}), '
                f'z={self.position.z:.2f} '
                f'(target={self.target_z:.2f}), '
                f'ARMED={self.is_armed()}, '
                f'OFFBOARD={self.is_offboard()}'
            )

    # ================================================================
    # Main state machine
    # ================================================================

    def timer_callback(self):

        # Always publish the current setpoint - PX4 needs a
        # continuous OFFBOARD-capable setpoint stream flowing before
        # it will accept the OFFBOARD mode switch, so this keeps
        # running even while idle on the ground.

        self.publish_offboard_control()
        self.publish_setpoint(self.target_x, self.target_y, self.target_z)

        if self.state == 'idle':
            return

        if self.position is None or self.status is None:

            self.get_logger().info(
                'Waiting for PX4 position/status...',
                throttle_duration_sec=2.0
            )

            return

        now = (
            self.get_clock()
            .now()
            .nanoseconds / 1e9
        )

        if now - self.last_status_print > 5.0:

            self.print_status()

            self.last_status_print = now

        if self.state == 'taking_off':

            if not self.is_offboard():

                self.request_offboard()

                return

            if not self.is_armed():

                self.request_arm()

                return

            self.target_z = max(
                self.target_z - self.climb_rate * self.dt,
                self.hover_z
            )

            if abs(self.position.z - self.hover_z) < 0.1:

                self.state = 'hovering'

                self.get_logger().info(
                    '========================================'
                )

                self.get_logger().info(
                    f'HOVER REACHED at {-self.hover_z:.2f}m'
                )

                self.get_logger().info(
                    'Waiting for command from hover_commander: '
                    '2 = land'
                )

                self.get_logger().info(
                    '========================================'
                )

        elif self.state == 'hovering':

            self.target_z = self.hover_z

            if self.target_x < self.goal_x:
                self.target_x = min(
                    self.target_x + self.horizontal_speed * self.dt,
                    self.goal_x
                )
            elif self.target_x > self.goal_x:
                self.target_x = max(
                    self.target_x - self.horizontal_speed * self.dt,
                    self.goal_x
                )

            if self.target_y < self.goal_y:
                self.target_y = min(
                    self.target_y + self.horizontal_speed * self.dt,
                    self.goal_y
                )
            elif self.target_y > self.goal_y:
                self.target_y = max(
                    self.target_y - self.horizontal_speed * self.dt,
                    self.goal_y
                )

        elif self.state == 'landing':

            self.target_z = min(
                self.target_z + self.land_descent_rate * self.dt,
                0.0
            )

            if self.position.z > -0.15:

                self.send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    0.0
                )

                self.get_logger().info(
                    'Landed: disarm command sent'
                )

                self.state = 'idle'

                self.target_z = 0.0

    # ================================================================
    # Land on shutdown (Ctrl+C), regardless of what the commander
    # last requested - descend to the ground and disarm before the
    # process exits.
    # ================================================================

    def emergency_land(self):

        if self.state == 'idle':
            return

        self.get_logger().info(
            'Shutdown requested: landing before exit'
        )

        z = (
            self.position.z
            if self.position is not None
            else self.target_z
        )

        while rclpy.ok() and z < -0.15:

            z = min(z + self.land_descent_rate * self.dt, 0.0)

            self.publish_offboard_control()
            self.publish_setpoint(self.target_x, self.target_y, z)

            rclpy.spin_once(self, timeout_sec=self.dt)

        if self.is_armed():

            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                0.0
            )

            self.get_logger().info('Disarm command sent')


def main(args=None):

    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = SingleUAVHover()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        try:

            node.emergency_land()

        except KeyboardInterrupt:

            node.get_logger().warn(
                'Landing interrupted, shutting down immediately'
            )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
