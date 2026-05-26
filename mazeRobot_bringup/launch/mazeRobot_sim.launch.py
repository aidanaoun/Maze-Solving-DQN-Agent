from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = get_package_share_directory('mazeRobot_bringup')
    urdf_path = os.path.join(pkg_share, 'urdf', 'mazerobot.urdf')

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    world_path = os.path.join(pkg_share, 'models', 'experiment_world.sdf')  # ensure this matches your file name

    # Fortress uses IGN_GAZEBO_RESOURCE_PATH
    set_env_vars_resources = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', os.path.join(pkg_share, 'models'))

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}])

    gazebo_launch_object = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_path}',
            'on_exit_shutdown': 'true'
        }.items())

    bridge_params = os.path.join(pkg_share, 'params', 'gazebo_ros_bridge.yaml')

    ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p', f'config_file:={bridge_params}',
        ],
        output='screen')

    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}])

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'maze_robot', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '0.35'],
        output='screen')

    ld.add_action(joint_state_pub_node)
    ld.add_action(robot_state_pub_node)
    ld.add_action(set_env_vars_resources)
    ld.add_action(gazebo_launch_object)
    ld.add_action(ros_bridge_node)
    ld.add_action(spawn_entity_node)

    return ld