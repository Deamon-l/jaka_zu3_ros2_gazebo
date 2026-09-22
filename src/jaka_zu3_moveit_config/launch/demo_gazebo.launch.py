from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    enable_camera = LaunchConfiguration("enable_camera")
    run_grasp = LaunchConfiguration("run_grasp")
    execute_grasp = LaunchConfiguration("execute_grasp")
    transfer_cycles = LaunchConfiguration("transfer_cycles")
    place_x = LaunchConfiguration("place_x")
    place_y = LaunchConfiguration("place_y")
    place_z = LaunchConfiguration("place_z")
    vision_params_file = LaunchConfiguration("vision_params_file")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    fastdds_udp_profile = PathJoinSubstitution(
        [
            FindPackageShare("jaka_zu3_moveit_config"),
            "config",
            "fastdds_udp.xml",
        ]
    )

    # 1. 载入 MoveIt 配置，开启 Gazebo 硬件模式
    moveit_config = (
        MoveItConfigsBuilder("jaka_zu3", package_name="jaka_zu3_moveit_config")
        .robot_description(
            file_path="config/jaka_zu3.urdf.xacro",
            mappings={
                "use_gazebo": "true",
                "use_rviz_sim": "false",
                "enable_camera_sensor": enable_camera,
                "enable_grasp_attachment": run_grasp,
            },
        )
        .to_moveit_configs()
    )

    # 2. Start the Gazebo server first.  Starting the GUI at the same time can
    # make EGL/shader initialisation block the simulation update loop before
    # gz_ros2_control has created controller_manager.
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [
                "-s -r ",
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

    # Attach the GUI independently of controller startup.  The server gets a
    # short head start, but a controller error must never prevent the user
    # from opening Gazebo and seeing its diagnostics.
    gazebo_gui = ExecuteProcess(
        cmd=["ign", "gazebo", "-g"],
        condition=UnlessCondition(headless),
        output="screen",
    )
    delayed_gazebo_gui = TimerAction(
        period=3.0,
        actions=[gazebo_gui],
    )

    # 3. 在 Ignition 中生成机械臂实体
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "jaka_zu3"],
        output="screen",
    )
    # robot_state_publisher and Gazebo are started together.  Give the ROS
    # parameter service time to become discoverable before gz_ros2_control
    # requests robot_description; otherwise the control plugin can wait
    # forever and controller_manager is never created.
    delayed_spawn_entity = TimerAction(
        period=2.0,
        actions=[spawn_entity],
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

    grasp_attachment_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=IfCondition(run_grasp),
        arguments=[
            "/gripper/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/gripper/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/gripper/attached@std_msgs/msg/String[ignition.msgs.StringMsg",
        ],
        output="screen",
    )

    target_detector = Node(
        package="jaka_zu3_vision",
        executable="target_detector",
        condition=IfCondition(enable_camera),
        parameters=[vision_params_file, {"use_sim_time": True}],
        output="screen",
    )

    target_motion_node = Node(
        package="jaka_zu3_vision",
        executable="target_motion",
        condition=IfCondition(run_grasp),
        parameters=[
            vision_params_file,
            {
                "use_sim_time": True,
                "execute_motion": ParameterValue(
                    execute_grasp, value_type=bool
                ),
                "transfer_cycles": ParameterValue(
                    transfer_cycles, value_type=int
                ),
                "place_x": ParameterValue(place_x, value_type=float),
                "place_y": ParameterValue(place_y, value_type=float),
                "place_z": ParameterValue(place_z, value_type=float),
                "use_sim_attachment": True,
            },
        ],
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
        condition=IfCondition(use_rviz),
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
    # RViz and move_group are also independent of controller activation.
    # MoveIt can wait for its action servers while the UI remains available.
    delayed_moveit_ui = TimerAction(
        period=3.0,
        actions=[move_group, rviz],
    )

    return LaunchDescription(
        [
            # false keeps the camera body and TF, but disables its renderer and bridge.
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="Enable the RGB-D sensors and detector.",
            ),
            DeclareLaunchArgument(
                "run_grasp",
                default_value="false",
                description="Start the guarded target-motion node.",
            ),
            DeclareLaunchArgument(
                "execute_grasp",
                default_value="false",
                description="Execute trajectories instead of plan-only mode.",
            ),
            DeclareLaunchArgument(
                "transfer_cycles",
                default_value="1",
                description=(
                    "Number of pick-and-place transfers; 2 performs a "
                    "round trip."
                ),
            ),
            DeclareLaunchArgument(
                "place_x",
                default_value="0.35",
                description="Place-point X in the target frame.",
            ),
            DeclareLaunchArgument(
                "place_y",
                default_value="-0.15",
                description="Place-point Y in the target frame.",
            ),
            DeclareLaunchArgument(
                "place_z",
                default_value="0.05",
                description="Placed object's top-surface Z.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run only the Gazebo server.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the MoveIt configuration.",
            ),
            DeclareLaunchArgument(
                "vision_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("jaka_zu3_vision"),
                        "config",
                        "vision_grasp.yaml",
                    ]
                ),
            ),
            # Disable Fast DDS shared memory for this launch. Interrupted
            # simulator runs otherwise leave stale SHM ports that can block
            # every subsequent ROS node before it prints its first log line.
            SetEnvironmentVariable(
                name="FASTRTPS_DEFAULT_PROFILES_FILE",
                value=fastdds_udp_profile,
            ),
            ign_gazebo,
            delayed_gazebo_gui,
            clock_bridge,
            camera_bridge,
            camera_info_bridge,
            grasp_attachment_bridge,
            rsp,
            delayed_spawn_entity,
            delayed_moveit_ui,
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
                    on_exit=[target_motion],
                )
            ),
        ]
    )
