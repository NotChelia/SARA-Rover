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
}

run_bridge_with_restart() {
    MAX_ATTEMPTS=10
    ATTEMPT=1
    
    while [ $ATTEMPT -le $MAX_ATTEMPTS ]; do
        echo "Bridge start attempt $ATTEMPT of $MAX_ATTEMPTS"
        ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &
        BRIDGE_PID=$!
        
        echo "Waiting for bridge..."
        sleep 2
        echo "ROS 1 topics:"
        rostopic list
        echo ""
        echo "ROS 2 topics:"
        ros2 topic list
        
        sleep 2
        
        if ps -p $BRIDGE_PID > /dev/null; then
            wait $BRIDGE_PID
        else
            echo "Bridge failed to start"
        fi
        
        ATTEMPT=$((ATTEMPT+1))
        
        sleep 5

    done
    
}

source_environments
run_bridge_with_restart
