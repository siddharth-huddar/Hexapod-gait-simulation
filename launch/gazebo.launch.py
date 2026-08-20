from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths
    hexapod_description_path = os.path.join(get_package_share_directory('hexapod'))
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    # Launch arguments
    x_arg = DeclareLaunchArgument(name='x', default_value='0')
    y_arg = DeclareLaunchArgument(name='y', default_value='0')
    z_arg = DeclareLaunchArgument(name='z', default_value='0')
    world_name_arg = DeclareLaunchArgument(
        name='world_name',
        default_value=os.path.join(hexapod_description_path, 'worlds', 'empty.world')
    )

    # Load the URDF
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        os.path.join(hexapod_description_path, 'urdf', 'hexapod.xacro')
    ])

    # Spawn the URDF in Gazebo
    spawn_urdf_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-param', 'robot_description',
            '-urdf',
            '-model', 'hexapod',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen'
    )

    # Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world_name'),
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )

    # Odom publisher node
    odom_publisher_node = Node(
        package='hexapod',
        executable='odom_publisher.py',
        output='screen'
    )

    # Controller configuration
    controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_hind_leg_joint_position_controller',
            'right_mid_leg_joint_position_controller',
            'right_front_leg_joint_position_controller',
            'left_hind_leg_joint_position_controller',
            'left_mid_leg_joint_position_controller',
            'left_front_leg_joint_position_controller',
            'joint_state_controller'
        ],
        output='screen',
        namespace='/hexapod'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        remappings=[('/joint_states', '/hexapod/joint_states')]
    )

    # Return the complete launch description
    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        world_name_arg,
        spawn_urdf_node,
        gazebo,
        odom_publisher_node,
        controller_spawner_node,
        robot_state_publisher_node
    ])
