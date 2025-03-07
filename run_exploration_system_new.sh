show_usage() {
    echo "open multiple terminal windows to run all components"
    echo ""
    echo "  Terminal 1: ./run_exploration_system_new.sh roscore"
    echo "  Terminal 2: ./run_exploration_system_new.sh lidar"
    echo "  Terminal 3: ./run_exploration_system_new.sh bridge"
    echo "  Terminal 4: ./run_exploration_system_new.sh pointlio"
    echo "              ./run_exploration_system_new.sh pointlio_minimal_rviz"
    echo "              ./run_exploration_system_new.sh pointlio_no_rviz"
    echo "  Terminal 5: ./run_exploration_system_new.sh explorer"
    echo ""
    echo "Setup:"
    echo "  ./run_exploration_system_new.sh build"

}

source_environments() {
    source /opt/ros/noetic/setup.bash
    source ~/Desktop/Lidar/catkin_point_lio_unilidar/devel/setup.bash
    
    source /opt/ros/foxy/setup.bash
    
    #change these paths
    if [ -f ~/Desktop/Lidar/exploration_ws/install/setup.bash ]; then
        source ~/Desktop/Lidar/exploration_ws/install/setup.bash
    fi
    
    if [ -f ~/Desktop/Lidar/unilidar_sdk/unitree_lidar_ros2/install/setup.bash ]; then
        source ~/Desktop/Lidar/unilidar_sdk/unitree_lidar_ros2/install/setup.bash
    fi
    
    export ROS_MASTER_URI=http://localhost:11311
    
    echo "ROS_MASTER_URI is set to: $ROS_MASTER_URI"
}

build_explorer() {
    cd ~/Desktop/Lidar/exploration_ws
    
    source /opt/ros/foxy/setup.bash
    
    colcon build --symlink-install --packages-select autonomous_explorer
    
    echo "package built"
    show_usage
}

run_roscore() {
    source /opt/ros/noetic/setup.bash
    roscore
}

run_lidar() {
    source_environments
    ros2 launch unitree_lidar_ros2 launch.py
}

run_bridge() {
    source_environments
    ~/Desktop/Lidar/robust_bridge.sh
}

run_pointlio() {
    source_environments
    roslaunch point_lio_unilidar mapping_unilidar_l1.launch
}

run_pointlio_minimal_rviz() {
    source_environments
    roslaunch point_lio_unilidar mapping_unilidar_l1_custom_rviz.launch rviz_cfg:=loam_livox_with_markers.rviz
}

run_pointlio_no_rviz() {
    source_environments
    roslaunch point_lio_unilidar mapping_unilidar_l1.launch rviz:=false
}

run_explorer() {
    source_environments
    ros2 run autonomous_explorer exploration_node_new
}

if [ $# -eq 0 ]; then
    show_usage
    exit 0
fi

case "$1" in
    "build")
        build_explorer
        ;;
    "roscore")
        run_roscore
        ;;
    "lidar")
        run_lidar
        ;;
    "bridge")
        run_bridge
        ;;
    "pointlio")
        run_pointlio
        ;;
    "pointlio_minimal_rviz")
        run_pointlio_minimal_rviz
        ;;
    "pointlio_no_rviz")
        run_pointlio_no_rviz
        ;;
    "explorer")
        run_explorer
        ;;
    *)
        echo "Unknown command: $1"
        show_usage
        exit 1
        ;;
esac
