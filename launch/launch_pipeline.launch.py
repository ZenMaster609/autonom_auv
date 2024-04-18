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
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    #pick world
    world_file_path = os.path.join(get_package_share_directory('autonom_auv'), 'worlds', 'pipeline.world')
    #configure Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file_path}.items()
             )

    #spawn ROV
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot','-x','15', '-z', '0.8',"-y","10"],
                        output='screen')

   
    #configure/include other nodes
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

  #  valve_image_node = Node(
   #     package = package_name,
    #    executable= 'valve_image_node',
     #   parameters=[{'save_images': True}],
      #  output='screen'
    #)

    m_pipeline_node = Node(
        package = package_name,
        executable= 'm_pipeline_node',
        output='screen'
    )
    

    movement_node = Node(
        package = package_name,
        executable= 'movement_node',
        output='screen'
    )
    
    dvl_movement_node = Node(
        package = package_name,
        executable= 'dvl_movement_node',
        output='screen'
    )

    
     #launch nodes
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        m_pipeline_node,
        movement_node,
        dvl_movement_node,
    ])
