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
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    world_file_path = os.path.join(get_package_share_directory('autonom_auv'), 'worlds', 'empty.world')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file_path}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    applyForce_script = Node(
        package=package_name,
        executable='applyForce',
        output='screen'
    )

    controller_script = Node(
        package=package_name,
        executable='controller',
        output='screen'
    )

    fakeController_script = Node(
        package=package_name,
        executable='fakeController',
        output='screen'
    )

    relativeForce_script = Node(
        package=package_name,
        executable='relativeForce',
        output='screen'
    )

    imageHandler_script = Node(
        package = package_name,
        executable= 'imageHandler',
        output='screen'
    )

    movement_script = Node(
        package = package_name,
        executable= 'movement',
        output='screen'
    )
    
 
    

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        movement_script,
        imageHandler_script
        #applyForce_script,
        #controller_script,
        #fakeController_script,
        #relativeForce_script
    ])
