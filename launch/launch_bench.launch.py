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

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'log_level': 'WARN'}.items()
    )
    
    world_file_path = os.path.join(get_package_share_directory('autonom_auv'), 'worlds', 'bench.world')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file_path, 'log_level': 'WARN'}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-z', '1', '--ros-args', '--log-level', 'WARN'], output='screen')

    fake_controller_node = Node(
        package = package_name,
        executable= 'fake_controller_node',
        output='screen'
    )

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

    fake_dvl_node = Node(
        package = package_name,
        executable= 'fake_dvl_node',
        output='screen'
    )

    movement_node = Node(
        package = package_name,
        executable= 'movement_node',
        output='screen'
    )
    
    
 
    

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        movement_node,
        m_bench_node,
        up_down_node,
        fake_dvl_node
    ])
