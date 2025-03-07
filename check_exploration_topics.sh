source /opt/ros/noetic/setup.bash
source ~/Desktop/Lidar/catkin_point_lio_unilidar/devel/setup.bash
source /opt/ros/foxy/setup.bash
source ~/Desktop/Lidar/exploration_ws/install/setup.bash

export ROS_MASTER_URI=http://localhost:11311

ros2 topic info /exploration/visualization_markers -v

rostopic info /exploration/visualization_markers

ros2 topic echo /exploration/visualization_markers --once 2>/dev/null | grep -A 2 "ns:"
rostopic echo /exploration/visualization_markers -n 1 2>/dev/null | grep -A 2 "ns:"

echo "All ROS 1 topics:"
rostopic list | grep exploration
echo ""

echo "All ROS 2 topics:"
ros2 topic list | grep exploration
