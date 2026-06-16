from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    receiver_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), 'sensors_udp_receiver.py')

    return LaunchDescription([

        ExecuteProcess(
            cmd=['python3', '-u', receiver_path],
            output='screen'
        ),

        # TF Tree Anchor: LiDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen'
        ),

        # TF Tree Anchor: IMU
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # sensor fusion engine
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'frequency': 30.0,
                'sensor_timeout': 0.1,
                'two_d_mode': True,
                'publish_tf': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                
                # [X, Y, Z, Roll, Pitch, Yaw, Vx, Vy, Vz, Vroll, Vpitch, Vyaw, Ax, Ay, Az]
                'odom0': '/wheel/odometry',
                'odom0_config': [True, True,  False, False, False, True,
                                 True, False, False, False, False, True,
                                 False, False, False],
                                 
                'imu0': '/imu/data',
                'imu0_config': [False, False, False, False, False, False,
                                False, False, False, False, False, True,
                                False, False, False]
            }]
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'minimum_travel_distance': 0.05,
                'minimum_travel_heading': 0.05
            }]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])