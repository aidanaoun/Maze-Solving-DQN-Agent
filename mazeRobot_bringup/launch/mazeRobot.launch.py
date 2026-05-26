from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    motor_controller = Node(
        package='mazeRobot',
        executable='motor_controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}])

    sensor_reader = Node(
        package='mazeRobot',
        executable='sensors_node',
        output='screen',
        parameters=[{'use_sim_time': True}])

    pose_reader = Node(
        package = 'mazeRobot',
        executable = 'pose_reader_node',
        output='screen',
        parameters=[{'use_sim_time': True}])

    brain = Node(
        package='mazeRobot',
        executable='brain_node',
        output='screen',
        parameters=[{'use_sim_time': True}])

    ld.add_action(pose_reader)
    ld.add_action(motor_controller)
    ld.add_action(sensor_reader)
    ld.add_action(brain)

    return ld