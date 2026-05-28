from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '/home/mongoose/Documents/EE2-Balance-Robot-LiDAR/lidar_udp_receiver.py'], # replace with your own filepath
            output='screen'
        ),

        # TF Tree Anchor (connects the LiDAR to the center of the robot)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen'
        ),

        # odometry
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic' : '/scan',
                'odom_topic' : '/odom',
                'publish_tf' : True,
                'base_frame_id' : 'base_link',
                'odom_frame_id' : 'odom',
                'init_pose_from_topic' : '',
                'freq' : 20.0
            }]
        ),

        # slam toolkit
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
        
        # visualiser
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])