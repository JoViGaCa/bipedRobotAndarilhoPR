import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
 
from launch_ros.actions import Node


 
def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    
    urdf_file_path = '/home/batman/Documentos/ICRobotica2/ROS2_Worskpace2/robo.xml'
    with open(urdf_file_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()
    
    robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                                parameters=[{
                                       'robot_description':urdf_content
                                       }],
                                output='screen')
 
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')
 
 
 
 
    # Run the node
    return LaunchDescription([
        robot_state_publisher,
        gazebo, 
        spawn_entity
    ])