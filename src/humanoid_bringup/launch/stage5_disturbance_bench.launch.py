"""Stage 5: fault-aware ROS control and CSV logging experiment.

Includes safety supervisor, fault topics, mission sequencing and logger.
No push recovery or unsupported dynamic walking is demonstrated. To
inject a delay from another shell, publish Float64 to the absolute topic
/mujoco_bridge/inject_imu_delay while this launch is running.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    num_steps_arg = DeclareLaunchArgument("num_steps", default_value="6")
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")
    autostart_delay_arg = DeclareLaunchArgument("autostart_delay_s", default_value="2.0")
    autostart_walk_delay_arg = DeclareLaunchArgument("autostart_begin_walk_delay_s", default_value="2.0")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")
    use_gui_arg = DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="false")
    model_file_arg = DeclareLaunchArgument("model_file", default_value="humanoid_biped_supported.xml")
    log_path_arg = DeclareLaunchArgument("log_path", default_value="/tmp/humanoid_lab_log.csv")

    stage4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("humanoid_bringup"), "launch", "stage4_walking.launch.py"])
        ),
        launch_arguments={
            "num_steps": LaunchConfiguration("num_steps"),
            "model_file": LaunchConfiguration("model_file"),
            "autostart": LaunchConfiguration("autostart"),
            "autostart_delay_s": LaunchConfiguration("autostart_delay_s"),
            "autostart_begin_walk_delay_s": LaunchConfiguration("autostart_begin_walk_delay_s"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_joint_state_publisher_gui": LaunchConfiguration("use_joint_state_publisher_gui"),
        }.items(),
    )

    log_recorder = Node(
        package="humanoid_benchmarks",
        executable="log_recorder",
        name="log_recorder",
        output="screen",
        parameters=[{"log_path": LaunchConfiguration("log_path")}],
    )

    return LaunchDescription([
        num_steps_arg, autostart_arg, autostart_delay_arg, autostart_walk_delay_arg,
        use_rviz_arg, use_gui_arg, model_file_arg, log_path_arg,
        stage4, log_recorder,
    ])
