"""Stage 1: load the URDF, publish TF, and view it in RViz2.
Run with: ros2 launch humanoid_description display.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui_arg = DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="true")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("humanoid_description"), "urdf", "humanoid.urdf.xacro"]
    )
    robot_description = Command(["xacro ", urdf_path])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_joint_state_publisher_gui")),
    )

    rviz_config = PathJoinSubstitution([FindPackageShare("humanoid_description"), "rviz", "display.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        use_gui_arg, use_rviz_arg,
        robot_state_publisher, joint_state_publisher_gui, rviz,
    ])
