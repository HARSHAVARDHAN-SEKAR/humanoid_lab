
"""Stage 4: Mission-controlled supported humanoid stepping.

The mission manager is responsible for starting the gait planner.

The Stage 3 independent gait autostart is explicitly disabled to
avoid competing startup commands.

The default MuJoCo model has a fixed/supported torso.
This demonstrates mission sequencing and controlled leg motion,
not dynamically balanced free-standing walking.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------

    num_steps_arg = DeclareLaunchArgument(
        "num_steps",
        default_value="4",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
    )

    autostart_delay_arg = DeclareLaunchArgument(
        "autostart_delay_s",
        default_value="2.0",
    )

    autostart_walk_delay_arg = DeclareLaunchArgument(
        "autostart_begin_walk_delay_s",
        default_value="2.0",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
    )

    use_gui_arg = DeclareLaunchArgument(
        "use_joint_state_publisher_gui",
        default_value="false",
    )

    model_file_arg = DeclareLaunchArgument(
        "model_file",
        default_value="humanoid_biped_supported.xml",
    )

    # --------------------------------------------------
    # Configuration file
    # --------------------------------------------------

    params_file = PathJoinSubstitution(
        [
            FindPackageShare("humanoid_bringup"),
            "config",
            "bringup_params.yaml",
        ]
    )

    # --------------------------------------------------
    # Include Stage 3 control stack
    # --------------------------------------------------

    stage3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("humanoid_bringup"),
                    "launch",
                    "stage3_stepping.launch.py",
                ]
            )
        ),
        launch_arguments={
            "num_steps": LaunchConfiguration("num_steps"),
            "model_file": LaunchConfiguration("model_file"),
            "safety_required": "true",
            "begin_walk_source": "/mission_manager/walk_requested",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_joint_state_publisher_gui": LaunchConfiguration(
                "use_joint_state_publisher_gui"
            ),
        }.items(),
    )

    # Stage 3 must not independently start walking.
    # Scoped configuration prevents its 0.0-second value
    # from changing the mission manager's delay.

    stage3_group = GroupAction(
        scoped=True,
        actions=[
            SetLaunchConfiguration(
                "autostart_delay_s",
                "0.0",
            ),
            stage3,
        ],
    )

    # --------------------------------------------------
    # Mission manager
    # --------------------------------------------------

    mission_manager = Node(
        package="humanoid_mission_manager",
        executable="mission_manager",
        name="mission_manager",
        output="screen",
        parameters=[
            params_file,
            {
                "num_steps": ParameterValue(
                    LaunchConfiguration("num_steps"),
                    value_type=int,
                ),
                "autostart": ParameterValue(
                    LaunchConfiguration("autostart"),
                    value_type=bool,
                ),
                "autostart_delay_s": ParameterValue(
                    LaunchConfiguration("autostart_delay_s"),
                    value_type=float,
                ),
                "autostart_begin_walk_delay_s": ParameterValue(
                    LaunchConfiguration(
                        "autostart_begin_walk_delay_s"
                    ),
                    value_type=float,
                ),
            },
        ],
    )

    # --------------------------------------------------
    # Safety supervisor
    # --------------------------------------------------

    safety_supervisor = Node(
        package="humanoid_safety",
        executable="safety_supervisor",
        name="safety_supervisor",
        output="screen",
        parameters=[params_file],
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------

    return LaunchDescription(
        [
            num_steps_arg,
            autostart_arg,
            autostart_delay_arg,
            autostart_walk_delay_arg,
            use_rviz_arg,
            use_gui_arg,
            model_file_arg,

            stage3_group,
            mission_manager,
            safety_supervisor,
        ]
    )

