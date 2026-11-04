"""Start MuJoCo physics and ROS sensor bridge. supported_base=true is a fixed torso rig."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution([FindPackageShare("humanoid_sim"), "mujoco_models",
                                   LaunchConfiguration("model_file")])
    # Choice of model path is driven by a Python expression-free launch
    # argument: supported is the safe, reproducible joint/stepping rig.
    return LaunchDescription([
        DeclareLaunchArgument("sim_publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("model_file", default_value="humanoid_biped_supported.xml"),
        DeclareLaunchArgument("mjcf_path", default_value=model),
        Node(package="humanoid_sim", executable="mujoco_bridge", name="mujoco_bridge",
             output="screen", parameters=[{
                 "sim_publish_rate_hz": ParameterValue(LaunchConfiguration("sim_publish_rate_hz"), value_type=float),
                 "mjcf_path": LaunchConfiguration("mjcf_path"),
             }]),
    ])
