import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    pkg_path = get_package_share_directory('adas_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'urdf_latest.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'adas_world.sdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # 1. Gazebo Harmonic - SERVER ONLY (-s = no GUI window, no GPU crash)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': os.path.join(
                    os.path.expanduser('~'),
                    'adas_ws/install/adas_robot_description/share'
                )
            }
        ),

        # 2. Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        # 3. Spawn robot at z=0.1 (wheels settle onto ground)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'adas_robot',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),

        # 4. Bridge Gazebo <-> ROS 2 topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/tof_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/tof_left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/tof_right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            output='screen'
        ),
    ])
