import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

 
 
from launch_ros.actions import Node

 
def generate_launch_description():
    #gazebo_world = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([
    #        '/home/batman/Documentos/ICRobotica3/gazebo_world.world'  # Replace with the actual path
    #    ])
    #)
    #DeclareLaunchArgument('world', description='/home/null/Documents/UTFPR/ICRobotica3/ROS2_Workspace/friction_world.world'),


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
           
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
    
    controller_node = Node(package='joint_publisher', executable='modelKasaei2019')


    rqt_plot = Node(package='rqt_plot', executable='rqt_plot',
                    arguments=['/center_of_mass/data', '/sensor_center_of_mass/data',
                                '/velocity_of_mass/data', '/sensor_velocity_of_mass/data',
                                '/acceleration_of_mass/data', '/sensor_acceleration_of_mass/data',
                                '/reference_value/data'])
 
 
 
 
    # Run the node
    return LaunchDescription([
        #gazebo_world,
        gazebo,
        robot_state_publisher,
        controller_node,
        rqt_plot,
        spawn_entity
        #joint_state_publisher_gui,
        #rviz2_node
    ])