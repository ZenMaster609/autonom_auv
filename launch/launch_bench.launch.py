import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable


def generate_launch_description():

    package_name='autonom_auv' 
    #Configure RSP
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'log_level': 'WARN'}.items()
    )
    #Pick world
    world_file_path = os.path.join(get_package_share_directory('autonom_auv'), 'worlds', 'bench.world')

    #Configure Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file_path, 'log_level': 'WARN'}.items()
             )

    #spawn ROV
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-z', '1', '--ros-args', '--log-level', 'WARN'], output='screen')


    #Configure/include other nodes
    up_down_node = Node(
        package = package_name,
        executable= 'up_down_node',
        output='screen'
    )

    m_bench_node = Node(
        package = package_name,
        executable= 'm_bench_node',
        output='screen'
    )

    dvl_movement_node = Node(
        package = package_name,
        executable= 'dvl_movement_node',
        output='screen'
    )

    movement_node = Node(
        package = package_name,
        executable= 'movement_node',
        output='screen'
    )
    
#Launch nodes
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        movement_node,
        up_down_node,
        dvl_movement_node,
        m_bench_node
    ])
