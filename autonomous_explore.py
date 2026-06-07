import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Autonomous frontier mapping: run ALONGSIDE launcher.py (which provides /map,
    /scan and TF). The explorer starts DISABLED; enable it with:

        ros2 service call /explore/enable std_srvs/srv/SetBool "{data: true}"

    Set the robot Pi's IP:  ros2 launch ./autonomous_explore.py pi_ip:=192.168.x.y
    """
    here = os.path.dirname(os.path.realpath(__file__))
    bridge = os.path.join(here, 'cmd_vel_bridge.py')
    explorer = os.path.join(here, 'frontier_explorer.py')

    pi_ip = LaunchConfiguration('pi_ip')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pi_ip', default_value='10.144.216.133',
            description="Robot Pi LAN IP for UDP velocity commands (client.py :31416)"),

        # /cmd_vel -> UDP -> Pi client.py cmd_listener_loop -> ESP32 V:/A:
        ExecuteProcess(
            cmd=['python3', '-u', bridge, '--ros-args', '-p', ['pi_ip:=', pi_ip]],
            output='screen'
        ),

        # /map + /scan + TF -> /cmd_vel (frontier exploration; DEFAULT DISABLED)
        ExecuteProcess(
            cmd=['python3', '-u', explorer],
            output='screen'
        ),
    ])
