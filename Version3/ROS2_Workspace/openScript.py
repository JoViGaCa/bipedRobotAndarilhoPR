import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

 
 
from launch_ros.actions import Node

 
def generate_launch_description():
    #gazebo_world = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([
    #        '/home/batman/Documentos/ICRobotica3/gazebo_world.world'  # Replace with the actual path
    #    ])
    #)
    #DeclareLaunchArgument('world', description='/home/null/Documents/UTFPR/ICRobotica3/ROS2_Workspace/friction_world.world'),

    world_file_arg = DeclareLaunchArgument('world', default_value='/home/null/Documents/UTFPR/ICRobotica3/ROS2_Workspace/gazebo_world.world',
                                          description='Path to the Gazebo world file')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
           launch_arguments={'world': LaunchConfiguration('world')}.items(),
        )
    
    
    urdf_file_path = '/home/null/Documents/UTFPR/ICRobotica3/ROS2_Workspace/andarilhoPR31.xml'
    with open(urdf_file_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()


    robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                                 parameters=[{
                                        'robot_description':urdf_content
                                        }],
                                output='screen')
    
    joint_state_publisher_gui = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'
                                    , output='screen')
    
    rviz2_node = Node(package='rviz2', executable='rviz2',
                        output='screen',
                        arguments=['-d', """
                            <launch>
                                <node name="robot_state" pkg="rviz2" type="robot" output="screen">
                                    <remap from="robot_description" to="/robot_description" />
                                    <param name="use_gui" value="true" />
                                </node>
                            </launch>
                        """])
                    
 
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                   arguments=['-topic', 'robot_description',
                                '-entity', 'AndarilhoPR3',
                                '-x', '0.0',   # Set the desired X position
                               '-y', '0.0',   # Set the desired Y position
                               '-z', '5',   # Set the desired Z position
                               '-Y', '0'],
                    output='screen')
 
 
 
 
    # Run the node
    return LaunchDescription([
        #gazebo_world,
        world_file_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity
        #joint_state_publisher_gui,
        #rviz2_node
    ])