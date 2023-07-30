import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        # load from package `ur5_robot_moveit_config`
        MoveItConfigsBuilder("ur5_robot", package_name="ur5_robot_moveit_config")
        # # Load robot description.
        # .robot_description(file_path="config/ec66.urdf.xacro")
        # # Load trajectory execution and moveit controller managers' parameters
        # .trajectory_execution(
        #     file_path="config/moveit_controllers.yaml"
        # )
        .planning_scene_monitor(  # modify default settings
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        # # Load planning pipelines parameters from `config/xxx_planning.yaml`
        # .planning_pipelines(
        #     pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        # )
        # Get a `MoveItConfigs` instance with all parameters loaded.
        .to_moveit_configs()
    )  # brackets can me omitted actually

    rviz_config_path = os.path.join(
        get_package_share_directory("movemove"), "config", "moveit.rviz"
    )  # or you can use: str(moveit_config.package_path / "config/moveit.rviz")
    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_path,
        description="RViz configuration file",
    )
    declared_arguments = [rviz_config]

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        respawn=True,  # dont know what means
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ## ros2_control related nodes (ros2_control using FakeSystem as hardware)
    ros2_controllers_path = str(
        moveit_config.package_path / "config/ros2_controllers.yaml"
    )  # ros2_controllers.yaml 中定义了后面启动的控制器
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # publish joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",  # 启动名为 joint_state_broadcaster 的控制器
            "--controller-manager-timeout",
            "300",
            "--controller-manager",  # -c 指定 controller manager
            "/controller_manager",  # controller manager 的名称为 /controller_manager
        ],
    )
    # load controllers
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ur5_arm_controller",  # 启动名为 ur5_arm_controller 的控制器
            "-c",  # -c 指定 controller manager
            "/controller_manager",  # 控制器在 /controller_manager 下被管理
        ],
    )

    nodes_to_start = [
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
