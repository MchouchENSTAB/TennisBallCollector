from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share1 = FindPackageShare('robot_tennis').find('robot_tennis')
    pkg_share2 = FindPackageShare('gazebo_ros').find('gazebo_ros')
    model_file = os.path.join(pkg_share1, 'urdf', "robot_tennis.urdf")
    #mesh_file = os.path.join(pkg_share1, 'meshs', "pince2.dae")
    path = os.path.join(pkg_share2, 'launch', "gazebo.launch.py")
    #rviz_config_file = os.path.join(pkg_share, 'config', "display.rviz")
    
    robot_state_publisher_node=Node(
        package="robot_state_publisher", node_executable="robot_state_publisher",
        arguments=[model_file]#, mesh_file]
        )
    rqt_robot_steering_node=Node(package="rqt_robot_steering", node_executable="rqt_robot_steering")
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([path]),
             )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                        arguments=['-entity', 'robot_tennis', '-topic', '/robot_description'])

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        rqt_robot_steering_node
    ])