from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    pkg_path = get_package_share_directory('assignment')
    world_path = os.path.join(pkg_path, 'worlds', 'world.world')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    #Launch Gazebo
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path],
        output='screen'
    )

    #Publish robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_path]
    )

    #Spawn robot 
    spawn_robot = TimerAction(
        period=5.0,  # wait 5 seconds
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', urdf_path,
                    '-name', 'robot',
                    '-x', '-2.5', '-y', '-2.5', '-z', '0.1', '-Y', '0.7845'
                ],
                output='screen'
            )
        ]
    )

    #Bridge Ignition and ROS2
    bridge = Node(
    package='ros_ign_bridge',
    executable='parameter_bridge',
    name='ros_ign_bridge',
    output='screen',
    arguments=[
        '/model/robot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        '/model/robot/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        '/world/simple_world/model/robot/link/base_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
    ],
    remappings=[
        ('/model/robot/cmd_vel', '/cmd_vel'),
        ('/model/robot/odom', '/odom'),
        ('/world/simple_world/model/robot/link/base_link/sensor/lidar/scan', '/scan')
    ])


    return LaunchDescription([
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
    ])