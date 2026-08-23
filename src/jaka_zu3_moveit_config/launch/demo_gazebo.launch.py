from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    enable_camera = LaunchConfiguration("enable_camera")
    run_grasp = LaunchConfiguration("run_grasp")
    execute_grasp = LaunchConfiguration("execute_grasp")

    # 1. 载入 MoveIt 配置，开启 Gazebo 硬件模式
    moveit_config = (
        MoveItConfigsBuilder("jaka_zu3", package_name="jaka_zu3_moveit_config")
        .robot_description(
            file_path="config/jaka_zu3.urdf.xacro",
            mappings={
                "use_gazebo": "true",
                "use_rviz_sim": "false",
                "enable_camera_sensor": enable_camera,
            },
        )
        .to_moveit_configs()
    )

    # 2. Start the regular Gazebo server and GUI with rendering sensors enabled.
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("jaka_zu3_moveit_config"),
                        "config",
                        "jaka_rgbd_world.sdf",
                    ]
                ),
            ]
        }.items(),
    )

    # 3. 在 Ignition 中生成机械臂实体
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "jaka_zu3"],
        output="screen",
    )

    # 4. Bridge simulation time immediately. Camera subscriptions are delayed
    # until all controllers are active so first-render shader setup cannot block them.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        condition=IfCondition(enable_camera),
        arguments=[
            "/wrist_camera/rgb_image",
            "/wrist_camera/depth_image",
        ],
        output="screen",
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=IfCondition(enable_camera),
        arguments=[
            "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    target_detector = Node(
        package="jaka_zu3_vision",
        executable="target_detector",
        condition=IfCondition(enable_camera),
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    target_motion_node = Node(
        package="jaka_zu3_vision",
        executable="target_motion",
        condition=IfCondition(run_grasp),
        parameters=[{
            "use_sim_time": True,
            "execute_motion": ParameterValue(execute_grasp, value_type=bool),
        }],
        output="screen",
    )
    # MoveIt and ros2_control need to finish initialization before the first
    # action goal is sent.  Starting immediately can race controller loading.
    target_motion = TimerAction(
        period=8.0,
        actions=[target_motion_node],
    )

    # 5. 发布机器人状态 (开启仿真时间)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # 6. Load all controllers before the first RGB-D render blocks the update loop,
    # then activate them in one switch operation.
    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "jaka_zu3_controller",
            "gripper_controller",
            "--activate-as-group",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "90",
            "--service-call-timeout",
            "90",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # 7. MoveGroup 规划核心
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    # 8. RViz2 界面
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("jaka_zu3_moveit_config"),
                    "config",
                    "moveit.rviz",
                ]
            ),
        ],
    )

    return LaunchDescription(
        [
            # false keeps the camera body and TF, but disables its renderer and bridge.
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument("run_grasp", default_value="false"),
            DeclareLaunchArgument("execute_grasp", default_value="false"),
            ign_gazebo,
            clock_bridge,
            camera_bridge,
            camera_info_bridge,
            rsp,
            spawn_entity,
            # Do not let TF consumers observe the Gazebo startup clock reset.
            # Spawn the robot first, then activate controllers, and only then
            # start MoveIt/RViz/vision nodes.
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[controllers_spawner, target_detector],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=controllers_spawner,
                    on_exit=[move_group, rviz, target_motion],
                )
            ),
        ]
    )
