import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Autonomous face seeking 
    # Need to run with launcher because it depends on /scan topic which only launcher publishes
    # ros2 service call /seek/enable std_srvs/srv/SetBool "{data: true}"
    here = os.path.dirname(os.path.realpath(__file__))
    seeker = os.path.join(here, 'face_seeker.py')
    bridge = os.path.join(here, 'cmd_vel_bridge.py')
    pi_ip = LaunchConfiguration('pi_ip')
    server = LaunchConfiguration('server')

    return LaunchDescription([
        DeclareLaunchArgument('pi_ip', default_value='10.232.9.80', description='Robot Pi LAN IP for UDP velocity commands'),
        DeclareLaunchArgument('server', default_value='10.232.9.34', description='Host running main.py (detections); laptop LAN IP'),
        ExecuteProcess(
            cmd=['python3', '-u', seeker, '--ros-args', '-p',['server_ws:=ws://', server, ':8000/ws/ui']],output='screen'),
        ExecuteProcess(
            cmd=['python3', '-u', bridge, '--ros-args', '-p', ['pi_ip:=', pi_ip]], output='screen'),
    ])
