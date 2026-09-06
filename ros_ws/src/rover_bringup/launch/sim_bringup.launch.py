from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mode = LaunchConfiguration("mode")  # modern | prop_m
    world = LaunchConfiguration("world")

    bringup_pkg = FindPackageShare("rover_bringup")
    mission_pkg = FindPackageShare("rover_mission_bt")
    description_pkg = FindPackageShare("rover_description")
    estimation_pkg = FindPackageShare("rover_estimation")
    sim_pkg = FindPackageShare("rover_sim_gazebo")

    common_params = PathJoinSubstitution([bringup_pkg, "params", "common.yaml"])
    modern_params = PathJoinSubstitution([bringup_pkg, "params", "modes", "modern.yaml"])
    prop_params = PathJoinSubstitution([bringup_pkg, "params", "modes", "prop_m.yaml"])
    mode_params = PythonExpression(['"', prop_params, '" if "', mode, '" == "prop_m" else "', modern_params, '"'])
    description_launch = PathJoinSubstitution([description_pkg, "launch", "description.launch.py"])
    estimation_launch = PathJoinSubstitution([estimation_pkg, "launch", "ekf.launch.py"])
    sim_launch = PathJoinSubstitution([sim_pkg, "launch", "sim.launch.py"])
    modern_tree = PathJoinSubstitution([mission_pkg, "trees", "modern_cycle.xml"])
    prop_tree = PathJoinSubstitution([mission_pkg, "trees", "prop_m_cycle.xml"])

    tree_file = PythonExpression([
        '"', prop_tree, '" if "', mode, '" == "prop_m" else "', modern_tree, '"'
    ])

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="modern", choices=["modern", "prop_m"]),
        DeclareLaunchArgument("world", default_value="mars_flat.sdf"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("start_mission", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "world": world,
                "seed": LaunchConfiguration("seed"),
                "gui": LaunchConfiguration("gui"),
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                "use_sim": "true",
                "use_ros2_control": "true",
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(estimation_launch),
            launch_arguments={
                "mode": mode,
            }.items()
        ),

        Node(
            package="rover_control",
            executable="rover_control_node",
            name="rover_control",
            output="screen",
            parameters=[common_params, mode_params, {"use_sim_time": True}],
        ),

        Node(
            package="rover_mission_bt",
            condition=IfCondition(LaunchConfiguration("start_mission")),
            executable="rover_mission_bt_node",
            name="rover_mission_bt",
            output="screen",
            parameters=[
                common_params,
                mode_params,
                {
                    "use_sim_time": True,
                    "tree_file": tree_file,
                },
            ],
        ),
    ])
