cd ~/ros2_ws/src
git clone https://github.com/Pier1102/lab02_bump-go.git
cd ~/ros2_ws
colcon build --packages-select lab02_pkg
source install/setup.bash
