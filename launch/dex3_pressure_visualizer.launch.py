from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = "dex3_pressure_tools"


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_robot_state_publisher = _as_bool(
        LaunchConfiguration("start_robot_state_publisher").perform(context)
    )
    start_rviz = _as_bool(LaunchConfiguration("start_rviz").perform(context))

    nodes = []

    if start_robot_state_publisher:
        urdf_path = PathJoinSubstitution([
            FindPackageShare(PACKAGE_NAME),
            "description_files",
            "urdf",
            "g1_29dof_dx3.urdf",
        ]).perform(context)
        with open(urdf_path, "r", encoding="utf-8") as stream:
            robot_description = stream.read()
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "robot_description": robot_description,
                }],
                arguments=[urdf_path],
            )
        )

    nodes.append(
        Node(
            package=PACKAGE_NAME,
            executable="dex3_pressure_visualizer",
            name="dex3_right_pressure_visualizer",
            output="screen",
            parameters=[{
                "state_topic": LaunchConfiguration("state_topic"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "mapping_file": LaunchConfiguration("mapping_file"),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float
                ),
                "baseline_samples": ParameterValue(
                    LaunchConfiguration("baseline_samples"), value_type=int
                ),
                "min_contact_delta": ParameterValue(
                    LaunchConfiguration("min_contact_delta"), value_type=float
                ),
                "full_scale_delta": ParameterValue(
                    LaunchConfiguration("full_scale_delta"), value_type=float
                ),
                "invalid_below": ParameterValue(
                    LaunchConfiguration("invalid_below"), value_type=float
                ),
                "invalid_value": ParameterValue(
                    LaunchConfiguration("invalid_value"), value_type=float
                ),
                "invalid_tolerance": ParameterValue(
                    LaunchConfiguration("invalid_tolerance"), value_type=float
                ),
                "show_labels": ParameterValue(
                    LaunchConfiguration("show_labels"), value_type=bool
                ),
                "enable_heatmap": ParameterValue(
                    LaunchConfiguration("enable_heatmap"), value_type=bool
                ),
                "publish_hand_joint_states": ParameterValue(
                    LaunchConfiguration("publish_hand_joint_states"), value_type=bool
                ),
                "publish_robot_joint_state_defaults": ParameterValue(
                    LaunchConfiguration("publish_robot_joint_state_defaults"), value_type=bool
                ),
            }],
        )
    )

    if start_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            )
        )

    return nodes


def generate_launch_description():
    package_share = FindPackageShare(PACKAGE_NAME)
    default_mapping = PathJoinSubstitution([
        package_share,
        "config",
        "dex3_right_pressure_seed.yaml",
    ])
    default_rviz = PathJoinSubstitution([
        package_share,
        "config",
        "dex3_pressure.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("state_topic", default_value="/lf/dex3/right/state"),
        DeclareLaunchArgument("marker_topic", default_value="/dex3/right/pressure_markers"),
        DeclareLaunchArgument("mapping_file", default_value=default_mapping),
        DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("baseline_samples", default_value="60"),
        DeclareLaunchArgument("min_contact_delta", default_value="50.0"),
        DeclareLaunchArgument("full_scale_delta", default_value="3000.0"),
        DeclareLaunchArgument("invalid_below", default_value="0.0"),
        DeclareLaunchArgument("invalid_value", default_value="30000.0"),
        DeclareLaunchArgument("invalid_tolerance", default_value="1000.0"),
        DeclareLaunchArgument("show_labels", default_value="true"),
        DeclareLaunchArgument("enable_heatmap", default_value="false"),
        DeclareLaunchArgument("publish_hand_joint_states", default_value="true"),
        DeclareLaunchArgument("publish_robot_joint_state_defaults", default_value="true"),
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        OpaqueFunction(function=_launch_setup),
    ])
