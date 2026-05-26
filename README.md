`lidar.py`: run on rspi

### ROS
`lidar_udp_sender.py`: run on rspi
`lidar_udp_receiver.py`: run on laptop

commands after to run program on laptop (new terminal):
`source /opt/ros/humble/setup.bash`
`python3 udp_lidar_receiver.py`


commands to show received json info (new terminal):
`source /opt/ros/humble/setup.bash`
`ros2 topic echo /scan`

launch ros2 to see map