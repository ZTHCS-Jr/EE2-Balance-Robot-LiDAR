import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    # Resolve helper scripts next to this launch file, so it works regardless of
    # username / checkout location.
    receiver_path = os.path.join(here, 'sensors_udp_receiver.py')
    wheel_odom_path = os.path.join(here, 'wheel_odometry_node.py')

    return LaunchDescription([

        # UDP -> ROS bridge: publishes /scan, /imu/data (yaw-rate only) and
        # /wheel/joint_states (raw stepper counts, stamped with the ESP32 clock).
        ExecuteProcess(
            cmd=['python3', '-u', receiver_path],
            output='screen'
        ),

        # Wheel odometry: /wheel/joint_states -> /wheel/odom (vx trusted, yaw not).
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

        # --- Sensor fusion: wheel vx + gyro yaw-rate -> odom -> base_link ---
        # rf2o laser odometry removed: scan-matching odometry can itself double
        # walls. Translation now comes from the wheels, heading from the gyro rate.
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

                # Wheel odometry: trust forward velocity (vx) ONLY. Let the EKF
                # integrate pose; wheel heading is not fused (its covariance is huge).
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
                'minimum_travel_distance': 0.05,  # 5 cm: responsive map updates. Safe now that
                                                  # wheel+gyro odometry is clean (the old 0.2 was
                                                  # to mask noisy rf2o/absolute-yaw odometry).
                'minimum_travel_heading': 0.1,    # ~5.7 deg; lower this too if turns map coarsely
                'minimum_time_interval': 0.1,     # throttle: <=5 scans/s processed (raise CPU floor)
                'min_laser_range': 0.1,           # match the scan range_min
                'max_laser_range': 8.0,           # match the scan range_max (and client clamp)
                'resolution': 0.05,
                'transform_timeout': 0.3,
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                # Loop closure / dedup. Raise loop_match_minimum_response_fine if you
                # see false closures; lower it (cautiously) if real loops are missed.
                'do_loop_closing': True,
                'loop_search_maximum_distance': 3.0,
                'loop_match_minimum_response_coarse': 0.35,
                'loop_match_minimum_response_fine': 0.45,
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
