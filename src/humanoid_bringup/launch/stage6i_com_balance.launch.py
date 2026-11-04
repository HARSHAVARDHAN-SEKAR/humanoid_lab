"""Stage 6I: dedicated free-base ROS2 CoM feedback experiment.

Uses Stage6H monitor stack; Stage5 motion_controller is NOT launched.
No walking planner, external push or model internals exposed to controller.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    observation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("humanoid_bringup"),
                "launch",
                "stage6h_com_observation.launch.py",
            ])
        )
    )
    estimator = Node(
        package="humanoid_state_estimator",
        executable="state_estimator",
        name="state_estimator",
        output="screen",
    )
    supervisor = Node(
        package="humanoid_safety",
        executable="safety_supervisor",
        name="safety_supervisor",
        output="screen",
    )
    controller = Node(
        package="humanoid_balance_controller",
        executable="com_balance_controller",
        name="com_balance_controller",
        output="screen",
    )
    return LaunchDescription([observation, estimator, supervisor, controller])
