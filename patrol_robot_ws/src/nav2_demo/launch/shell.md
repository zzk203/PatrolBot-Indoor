colcon build --packages-select nav2_demo
source install/setup.bash
ros2 launch nav2_demo patrol_sim.launch.py