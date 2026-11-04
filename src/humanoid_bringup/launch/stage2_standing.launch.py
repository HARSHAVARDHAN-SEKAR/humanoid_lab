"""Stage 2: supported-torso standing subsystem integration.

Composes stage1, IMU state estimator, ankle PD and the joint-reference
controller. The motion controller sends the 12 joint IK targets to MuJoCo.
A fixed-torso rig is default and cannot demonstrate unsupported standing.
Switch to experimental free base with model_file:=humanoid_biped.xml.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")
    use_gui_arg = DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="false")

    model_file_arg = DeclareLaunchArgument("model_file", default_value="humanoid_biped_supported.xml")
    safety_arg = DeclareLaunchArgument("safety_required", default_value="false")
    stage1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("humanoid_bringup"), "launch", "stage1_joint_control.launch.py"])
        ),
        launch_arguments={
            "model_file": LaunchConfiguration("model_file"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_joint_state_publisher_gui": LaunchConfiguration("use_joint_state_publisher_gui"),
        }.items(),
    )
    params_file = PathJoinSubstitution([FindPackageShare("humanoid_bringup"), "config", "bringup_params.yaml"])

    state_estimator = Node(
        package="humanoid_state_estimator",
        executable="state_estimator",
        name="state_estimator",
        output="screen",
    )
    balance_controller = Node(
        package="humanoid_balance_controller",
        executable="balance_controller",
        name="balance_controller",
        output="screen",
        parameters=[params_file],
    )

    motion_controller = Node(
        package="humanoid_motion_controller", executable="motion_controller",
        name="motion_controller", output="screen",
        parameters=[{"safety_required": ParameterValue(LaunchConfiguration("safety_required"), value_type=bool)}],
    )
    return LaunchDescription([use_rviz_arg, use_gui_arg, model_file_arg, safety_arg,
                              stage1, state_estimator, balance_controller, motion_controller])
