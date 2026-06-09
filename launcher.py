import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    # Paths to our ROS2 nodes
    receiver_path = os.path.join(here, 'sensors_udp_receiver.py')
    wheel_odom_path = os.path.join(here, 'wheel_odometry_node.py')

    return LaunchDescription([

        # Receives over UDP data fro Rpi and publishes /scan, /imu/data (yaw-rate only) and
        # /wheel/joint_states (raw stepper counts, stamped with the ESP32 clock)
        ExecuteProcess(
            cmd=['python3', '-u', receiver_path],
            output='screen'
        ),

        # Subscribes to joint_state and converts the raw motor steps into motor odom (linear/angular velocity)
        ExecuteProcess(
            cmd=['python3', '-u', wheel_odom_path],
            output='screen'
        ),

        # --- TF tree (tilt handling deferred -> flat): map -> odom -> base_link -> {laser_frame, imu_link} ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link'
            ],
            output='screen'
        ),
        
        # Sensor fusion: wheel velocity and gyro yaw-rate combined to create odom frame
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

                # Wheel odometry: trust forward velocity (vx) ONLY. The EKF
                # integrates the pose
                # We dong fuse wheel heading.
                # order: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
                'odom0': '/wheel/odom',
                'odom0_config': [False, False, False,
                                 False, False, False,
                                 True,  False, False,
                                 False, False, False,
                                 False, False, False],

                # IMU: trust yaw RATE (vyaw) ONLY. Absolute yaw is NOT fused, so the
                # heading never inherits the MPU's integration drift.
                'imu0': '/imu/data',
                'imu0_config': [False, False, False,
                                False, False, False,
                                False, False, False,
                                False, False, True,
                                False, False, False],
            }]
        ),

        # --- SLAM ---
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
                'map_update_interval': 0.3,        # republish /map ~3 Hz for snappier RViz refresh
                'minimum_travel_distance': 0.05,  # Update map every 5cm travelled
                'minimum_travel_heading': 0.05,   # ~2.9 deg (was 0.1/5.7); finer scan cadence in turns -> less doubling
                'minimum_time_interval': 0.1,     # throttle: <=5 scans/s processed (raise CPU floor)
                'min_laser_range': 0.1,           # matches the scan range_min
                'max_laser_range': 8.0,           # matches the scan range_max 
                'resolution': 0.05,
                'transform_timeout': 0.3,
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'do_loop_closing': True,
                'loop_search_maximum_distance': 3.0,
                'loop_match_minimum_response_coarse': 0.45,  # was 0.35: stricter pre-filter
                'loop_match_minimum_response_fine': 0.55,    # was 0.45: only accept a HIGH-
                                                             # confidence closure, so the
                                                             # whole-map snap lands right
                                                             # instead of leaving an angle
                'loop_match_minimum_chain_size': 12,         # require a longer agreeing chain
                                                             # of scans -> reject spurious loops
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
