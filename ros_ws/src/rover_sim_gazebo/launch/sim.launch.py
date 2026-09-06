from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world = LaunchConfiguration("world")

    pkg = FindPackageShare("rover_sim_gazebo")
    gazebo_pkg = FindPackageShare("gazebo_ros")

    gazebo_launch = PathJoinSubstitution([gazebo_pkg, "launch", "gazebo.launch.py"])
    world_path = PathJoinSubstitution([pkg, "worlds", world])

    return LaunchDescription([
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("world", default_value="mars_flat.sdf",
                             description="World file inside rover_sim_gazebo/worlds/"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"world": world_path, "seed": LaunchConfiguration("seed"), "gui": LaunchConfiguration("gui"), "server_required": "true"}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, "launch", "spawn_rover.launch.py"])
            ),
        ),
    ])
