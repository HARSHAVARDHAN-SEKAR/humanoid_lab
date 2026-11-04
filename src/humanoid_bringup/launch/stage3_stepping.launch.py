"""Stage 3: supported swing-foot references integrated with IK and actuators.

Adds the gait planner to stage2; defaults to no automatic start unless
`autostart_delay_s` is specified (make stage3 sets 3 seconds). `num_steps`
is 2 by default. This is fixed-torso stepping, NOT dynamic walking.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    num_steps_arg = DeclareLaunchArgument("num_steps", default_value="2")
    autostart_delay_arg = DeclareLaunchArgument("autostart_delay_s", default_value="0.0")
    begin_walk_source_arg = DeclareLaunchArgument("begin_walk_source", default_value="/gait_planner/begin_walk")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")
    use_gui_arg = DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="false")

    model_file_arg = DeclareLaunchArgument("model_file", default_value="humanoid_biped_supported.xml")
    safety_arg = DeclareLaunchArgument("safety_required", default_value="false")
    stage2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("humanoid_bringup"), "launch", "stage2_standing.launch.py"])
        ),
        launch_arguments={
            "model_file": LaunchConfiguration("model_file"),
            "safety_required": LaunchConfiguration("safety_required"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_joint_state_publisher_gui": LaunchConfiguration("use_joint_state_publisher_gui"),
        }.items(),
    )
    params_file = PathJoinSubstitution([FindPackageShare("humanoid_bringup"), "config", "bringup_params.yaml"])

    gait_planner = Node(
        package="humanoid_gait_planner",
        executable="gait_planner",
        name="gait_planner",
        output="screen",
        parameters=[params_file, {
            "num_steps": ParameterValue(LaunchConfiguration("num_steps"), value_type=int),
            "autostart_delay_s": ParameterValue(LaunchConfiguration("autostart_delay_s"), value_type=float),
        }],
        remappings=[("~/begin_walk", LaunchConfiguration("begin_walk_source"))],
    )

    return LaunchDescription([
        num_steps_arg, autostart_delay_arg, begin_walk_source_arg, use_rviz_arg, use_gui_arg,
        model_file_arg, safety_arg, stage2, gait_planner,
    ])
