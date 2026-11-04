"""Stage 6H: observe free-base CoM using ROS messages.

Monitoring only: no autonomous walking or balance commands.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    stage1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("humanoid_bringup"),
                "launch",
                "stage1_joint_control.launch.py",
            ])
        ),
        launch_arguments={
            "model_file": "humanoid_biped.xml",
            "use_rviz": "false",
            "use_joint_state_publisher_gui": "false",
        }.items(),
    )

    estimator = Node(
        package="humanoid_state_estimator",
        executable="com_estimator",
        name="com_estimator",
        output="screen",
    )

    return LaunchDescription([
        stage1,
        estimator,
    ])
