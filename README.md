# task_3














ros2 launch tod_nav2 display.launch.py 
ros2 run turtlebot3_teleop teleop_keyboard

ros2 run nav2_map_server map_saver_cli -f my_map






ros2 launch tod_nav2 gazebo_launch.py 
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=src/tod_nav2/maps/my_map.yaml
