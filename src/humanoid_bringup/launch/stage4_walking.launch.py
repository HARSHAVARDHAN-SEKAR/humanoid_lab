"""Stage 4: gait-planning and mission sequencing experiment.

Mission autostarts by default and sends short-stride swing references
through IK to MuJoCo actuators. The physics model is a fixed-torso rig
by default. Launch success is not evidence of dynamically stable walking.
For unsupported experimental physics use model_file:=humanoid_biped.xml.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    num_steps_arg = DeclareLaunchArgument("num_steps", default_value="6")
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")
    autostart_delay_arg = DeclareLaunchArgument("autostart_delay_s", default_value="2.0")
    autostart_walk_delay_arg = DeclareLaunchArgument("autostart_begin_walk_delay_s", default_value="2.0")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")
    use_gui_arg = DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="false")
    model_file_arg = DeclareLaunchArgument("model_file", default_value="humanoid_biped_supported.xml")

    stage3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("humanoid_bringup"), "launch", "stage3_stepping.launch.py"])
        ),
        launch_arguments={
            "num_steps": LaunchConfiguration("num_steps"),
            "model_file": LaunchConfiguration("model_file"),
            "safety_required": "true",
            "begin_walk_source": "/mission_manager/walk_requested",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_joint_state_publisher_gui": LaunchConfiguration("use_joint_state_publisher_gui"),
            # autostart_delay_s intentionally NOT forwarded here -- stays at
            # stage3's own default (0.0 = off), since mission_manager (below)
            # is what drives ~begin_walk once it's in the launch tree.
        }.items(),
    )
    params_file = PathJoinSubstitution([FindPackageShare("humanoid_bringup"), "config", "bringup_params.yaml"])

    mission_manager = Node(
        package="humanoid_mission_manager",
        executable="mission_manager",
        name="mission_manager",
        output="screen",
        parameters=[params_file, {
            "num_steps": ParameterValue(LaunchConfiguration("num_steps"), value_type=int),
            "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool),
            "autostart_delay_s": ParameterValue(LaunchConfiguration("autostart_delay_s"), value_type=float),
            "autostart_begin_walk_delay_s": ParameterValue(LaunchConfiguration("autostart_begin_walk_delay_s"), value_type=float),
        }],
    )
    safety_supervisor = Node(
        package="humanoid_safety",
        executable="safety_supervisor",
        name="safety_supervisor",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        num_steps_arg, autostart_arg, autostart_delay_arg, autostart_walk_delay_arg,
        use_rviz_arg, use_gui_arg, model_file_arg,
        stage3, mission_manager, safety_supervisor,
    ])
