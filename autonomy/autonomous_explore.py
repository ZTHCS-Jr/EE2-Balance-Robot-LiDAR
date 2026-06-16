import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Autonomous mapping: run ALONGSIDE launcher.py (which provides /scan, /map, TF,
    EKF /odometry/filtered). Brings up nav2 + explore_lite + the /cmd_vel->Pi bridge.

        ros2 launch ./autonomous_explore.py pi_ip:=192.168.x.y

    nav2 has NO amcl/map_server here -- slam_toolbox (from launcher.py) supplies map->odom.
    """
    here = os.path.dirname(os.path.realpath(__file__))
    nav2_params = os.path.join(here, 'nav2_params.yaml')
    explore_params = os.path.join(here, 'explore_params.yaml')
    bridge = os.path.join(here, 'cmd_vel_bridge.py')

    pi_ip = LaunchConfiguration('pi_ip')

    nav2_navigation = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pi_ip', default_value='10.232.9.80',
            description="Robot Pi LAN IP for UDP velocity commands (client.py :31416)"),

        # nav2 navigation stack (controller/planner/behavior/bt/costmaps/velocity_smoother).
        # No localization nodes -- slam_toolbox provides the map->odom transform.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': nav2_params,
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),

        # explore_lite: finds frontiers in the global costmap and sends nav2 goals.
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[explore_params],
        ),

        # nav2 /cmd_vel -> UDP {"v","w"} -> Pi client.py cmd_listener -> ESP32 V:/A:
        ExecuteProcess(
            cmd=['python3', '-u', bridge, '--ros-args', '-p', ['pi_ip:=', pi_ip]],
            output='screen',
        ),
    ])
