from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    package_share = FindPackageShare("jaka_zu3_moveit_config")

    # RViz uses the URDF/Xacro description; Gazebo below uses the native SDF model.
    robot_description = {
        "robot_description": Command([
            "xacro ",
            PathJoinSubstitution([package_share, "config", "test_camera.urdf.xacro"]),
        ])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "ign_args": [
                "-r ",
                PathJoinSubstitution([package_share, "config", "test_camera_world.sdf"]),
            ]
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name", "standalone_camera",
            "-file", PathJoinSubstitution([package_share, "config", "test_camera.sdf"]),
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/rgb_camera@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/depth_camera@sensor_msgs/msg/Image[ignition.msgs.Image",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz,
    ])
