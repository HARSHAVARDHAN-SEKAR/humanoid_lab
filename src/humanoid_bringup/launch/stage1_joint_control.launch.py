"""Stage 1: supported MuJoCo model, joint feedback and optional URDF/RViz.

Uses fixed-torso rig by default; free base via model_file:=humanoid_biped.xml.
No autonomous motion is requested in this stage. Headless by default.
Example single-joint command (without stage2 motion controller):
    ros2 topic pub --once /mujoco_bridge/joint_commands sensor_msgs/msg/JointState \
      "{name: ['right_hip_pitch_joint'], position: [0.2]}"
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")
    use_gui_arg = DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="false")

    model_file_arg = DeclareLaunchArgument("model_file", default_value="humanoid_biped_supported.xml")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("humanoid_sim"), "launch", "sim.launch.py"])
        ),
        launch_arguments={"model_file": LaunchConfiguration("model_file")}.items(),
    )
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("humanoid_description"), "launch", "display.launch.py"])
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_joint_state_publisher_gui": LaunchConfiguration("use_joint_state_publisher_gui"),
        }.items(),
    )
    return LaunchDescription([use_rviz_arg, use_gui_arg, model_file_arg, sim, display])
