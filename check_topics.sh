source /opt/ros/noetic/setup.bash
source ~/Desktop/Lidar/catkin_point_lio_unilidar/devel/setup.bash
source /opt/ros/foxy/setup.bash
source ~/Desktop/Lidar/exploration_ws/install/setup.bash
source ~/Desktop/Lidar/unilidar_sdk/unitree_lidar_ros2/install/setup.bash

export ROS_MASTER_URI=http://localhost:11311

check_topic() {
    local topic=$1
    echo "=== Checking topic: $topic ==="
    
    echo "ROS 1 Topic Info:"
    echo "----------------"
    rostopic info $topic
    
    echo "ROS 2 Topic Info:"
    echo "----------------"
    ros2 topic info $topic
    
    echo "ROS 1 Publishers and Subscribers:"
    echo "--------------------------------"
    rostopic info $topic
    
    echo "ROS 2 Publishers and Subscribers:"
    echo "--------------------------------"
    ros2 topic info $topic -v
}

check_topic "/pointlio/cloud_registered"
check_topic "/pointlio/odom"

check_topic "/exploration/visualization_markers"

echo "All ROS 1 Topics:"
echo "----------------"
rostopic list
echo ""

echo "All ROS 2 Topics:"
echo "----------------"
ros2 topic list
