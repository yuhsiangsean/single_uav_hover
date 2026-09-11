"""
實機部署用：每台 RPi 各跑一份，用 params_file 帶入自己的角色設定。

用法 (在 leader 的 RPi 上):
    ros2 launch formation_offboard single_node.launch.py \
        params_file:=/home/pi/ros2_ws/src/formation_offboard/params/leader.yaml \
        namespace:=MAV1

在 follower 的 RPi 上:
    ros2 launch formation_offboard single_node.launch.py \
        params_file:=/home/pi/ros2_ws/src/formation_offboard/params/follower_left.yaml \
        namespace:=MAV2

注意：三台 RPi 共用同一個 ROS_DOMAIN_ID，如果不給 namespace，三個 node 會用同一個名字
formation_offboard_control，`ros2 param set`/`ros2 node list` 會分不出是對哪一台下指令。
namespace 要跟 params_file 裡的 mav_id 對應（MAV1/MAV2/MAV3），節點名稱會變成
/MAV1/formation_offboard_control 這種可以明確指定的名字。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        description="Path to the role-specific params yaml (leader.yaml / follower_left.yaml / ...)",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        description="ROS2 namespace for this node, e.g. MAV1 / MAV2 / MAV3 - must match mav_id in params_file",
    )

    node = Node(
        package="formation_offboard",
        executable="formation_offboard_control",
        name="formation_offboard_control",
        namespace=LaunchConfiguration("namespace"),
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    return LaunchDescription([params_file_arg, namespace_arg, node])
