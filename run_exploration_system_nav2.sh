show_usage() {
    echo "  Terminal 1: ./run_exploration_system_nav2.sh roscore"
    echo "  Terminal 2: ./run_exploration_system_nav2.sh lidar"
    echo "  Terminal 3: ./run_exploration_system_nav2.sh bridge"
    echo "  Terminal 4: ./run_exploration_system_nav2.sh pointlio"
    echo "              ./run_exploration_system_nav2.sh pointlio_minimal_rviz"
    echo "              ./run_exploration_system_nav2.sh pointlio_no_rviz"
    echo "  Terminal 5: ./run_exploration_system_nav2.sh joy"
    echo "  Terminal 6: ./run_exploration_system_nav2.sh explorer"
    echo "  Terminal 7: ./run_exploration_system_nav2.sh serial"
}

source_environments() {
    source /opt/ros/noetic/setup.bash
    source ~/Desktop/Lidar/catkin_point_lio_unilidar/devel/setup.bash
    
    source /opt/ros/foxy/setup.bash
    
    if [ -f ~/Desktop/Lidar/exploration_ws/install/setup.bash ]; then
        source ~/Desktop/Lidar/exploration_ws/install/setup.bash
    fi
    
    if [ -f ~/Desktop/Lidar/unilidar_sdk/unitree_lidar_ros2/install/setup.bash ]; then
        source ~/Desktop/Lidar/unilidar_sdk/unitree_lidar_ros2/install/setup.bash
    fi
    
    export ROS_MASTER_URI=http://localhost:11311
    
    echo "ROS_MASTER_URI is set to: $ROS_MASTER_URI"
    echo ""
}

build_explorer() {
    cd ~/Desktop/Lidar/exploration_ws
    
    source /opt/ros/foxy/setup.bash
    
    colcon build --symlink-install --packages-select autonomous_explorer
    
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

run_joy() {
    source ~/Desktop/Lidar/exploration_ws/install/setup.bash
    
    ros2 launch autonomous_explorer joy.launch.py &
    JOY_PID=$!
    
    ros2 run autonomous_explorer cmd_vel_mux &
    MUX_PID=$!
    
    trap "echo 'Cleaning up joystick processes...'; kill $JOY_PID $MUX_PID" EXIT
    
    while true; do
        sleep 1
    done
}

run_explorer() {
    source ~/Desktop/Lidar/exploration_ws/install/setup.bash
    
    ros2 launch autonomous_explorer exploration_nav2.py
}

run_serial() {
    source ~/Desktop/Lidar/exploration_ws/install/setup.bash
    
    if [ ! -e "/dev/ttyTHS0" ]; then
        echo "Error: /dev/ttyTHS0 inai"
        exit 1
    fi
    
    python3 -c "import serial" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo "Error:'pyserial' inai"
        exit 1
    fi
    
    python3 ~/Desktop/Lidar/send_serial.py
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
    "joy")
        run_joy
        ;;
    "explorer")
        run_explorer
        ;;
    "serial")
        run_serial
        ;;
    *)
        echo "Unknown command: $1"
        show_usage
        exit 1
        ;;
esac
