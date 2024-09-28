import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
def generate_launch_description():
    ur5_moveit_config = MoveItConfigsBuilder("ur5_robot", package_name="ur5_robot_moveit_config").to_moveit_configs()
    ld = LaunchDescription()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("gazebo_ros"),'launch'), '/gazebo.launch.py']
        )
    )

    # package_path = os.path.join(get_package_share_directory('ur5_robot_moveit_config'))
    # print(package_path)
    # xacro_file = os.path.join(package_path,'config','ur5_robot.urdf.xacro')
    # print(xacro_file)
    # doc = xacro.parse(open(xacro_file))

    # xacro.process_doc(doc)
    # params = {'robot_description':doc.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[ur5_moveit_config.robot_description]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description', '-entity', 'ur5_robot'],
        output='screen'
    )

    ld.add_action(gazebo)

    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             PathJoinSubstitution([FindPackageShare("ur5_robot_moveit_config"), "launch", "demo.launch.py"]))
    #     )
    # )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity)

    return ld

