from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        
        ExecuteProcess(
            cmd=['python3', '-u', '/home/mongoose/Documents/EE2-Balance-Robot-LiDAR/sensors_udp_receiver.py'],
            output='screen'
        ),

        # TF Tree Anchor
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'
            ],
            output='screen'
        ),

        # laser odometry
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic' : '/scan',
                'odom_topic' : '/odom',
                'publish_tf' : False,
                'base_frame_id' : 'base_link',
                'odom_frame_id' : 'odom',
                'init_pose_from_topic' : '',
                'freq' : 20.0
            }]
        ),

        # sensor fusion engine
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'frequency': 50.0,
                'two_d_mode': True,
                'publish_tf': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                
                # LiDAR Odometry (Trust X/Y translation)
                'odom0': '/odom',
                'odom0_config' : [True,  True,  False, # trust X, Y, ignore Z
                                False, False, False, # ignore roll, pitch, yaw
                                False, False, False, # ignore X, Y, Z velocity
                                False, False, False, # ignore angular velocity
                                False, False, False], # ignore acceleration
                                 
                # IMU (Trust Yaw rotation/velocity only)
                'imu0': '/imu/data',
                'imu0_config': [False, False, False, # ignore X,Y,Z
                                False, False, True, #trust yaw
                                False, False, False, # ignore velocity
                                False, False, True, # trust yaw velocity
                                False, False, False] # ignore acceleration
            }]
        ),

        # SLAM toolkit
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
                'map_update_interval': 0.5,
                'minimum_travel_distance': 0.05,
                'minimum_travel_heading': 0.05,
                'use_scan_matching': True,
                'use_scan_barycenter': True,
            }]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])