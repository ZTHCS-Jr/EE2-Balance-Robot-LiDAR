import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Lightweight autonomous mapping (no nav2). Run ALONGSIDE launcher.py, which
    provides /scan, /map, TF and the EKF. This starts only:
      - frontier_explorer.py : reads /map + /scan, drives to frontiers, publishes /cmd_vel
      - cmd_vel_bridge.py    : /cmd_vel -> UDP {"v","w"} -> Pi client.py -> ESP32

    The explorer is gated OFF by default. After both launches are up:
        ros2 service call /explore/enable std_srvs/srv/SetBool "{data: true}"   # start
        ros2 service call /explore/enable std_srvs/srv/SetBool "{data: false}"  # stop

        ros2 launch ./explore_light.py pi_ip:=192.168.x.y
    """
    here = os.path.dirname(os.path.realpath(__file__))
    explorer = os.path.join(here, 'frontier_explorer.py')
    bridge = os.path.join(here, 'cmd_vel_bridge.py')

    pi_ip = LaunchConfiguration('pi_ip')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pi_ip', default_value='10.140.43.80',
            description="Robot Pi LAN IP for UDP velocity commands (client.py :31416)"),

        # Frontier explorer: /map + /scan -> /cmd_vel (no nav2, no global planner).
        ExecuteProcess(
            cmd=['python3', '-u', explorer],
            output='screen',
        ),

        # /cmd_vel -> UDP {"v","w"} -> Pi client.py cmd_listener -> ESP32 V:/A:
        ExecuteProcess(
            cmd=['python3', '-u', bridge, '--ros-args', '-p', ['pi_ip:=', pi_ip]],
            output='screen',
        ),
    ])
