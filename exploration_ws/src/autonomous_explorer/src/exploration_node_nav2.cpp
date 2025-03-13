#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <random>
#include <mutex>
#include <optional>

//nav2
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

/**
 * @brief Point in 2D grid 
 */
struct GridPoint {
    int x;
    int y;
    
    GridPoint() : x(0), y(0) {}
    GridPoint(int _x, int _y) : x(_x), y(_y) {}
    
    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y;
    }
    
    bool operator!=(const GridPoint& other) const {
        return !(*this == other);
    }
    
    double distanceTo(const GridPoint& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }
};

/**
 * @brief Point in world 
 */
struct WorldPoint {
    double x;
    double y;
    
    WorldPoint() : x(0.0), y(0.0) {}
    WorldPoint(double _x, double _y) : x(_x), y(_y) {}
    
    double distanceTo(const WorldPoint& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }
};

/**
 * @brief Frontier cluster, essentially a boundary between known/unknown space
 */
class Frontier {
public:
    std::vector<GridPoint> points;
    GridPoint centroid;
    WorldPoint world_centroid;
    double size = 0.0;
    double utility = 0.0;
    
    Frontier() : centroid(0, 0), world_centroid(0.0, 0.0) {}
    
    void calculateCentroid() {
        if (points.empty()) return;
        
        int sum_x = 0, sum_y = 0;
        for (const auto& point : points) {
            sum_x += point.x;
            sum_y += point.y;
        }
        
        centroid.x = sum_x / points.size();
        centroid.y = sum_y / points.size();
        size = points.size();
    }
};

/**
 * @brief robot state machine states
 */
enum class ExplorationState {
    INITIALIZING,
    EXPLORING,
    NAVIGATING_TO_FRONTIER,
    ROTATING_FOR_BETTER_VIEW,
    BACKTRACKING,
    RECOVERY,
    COMPLETED
};

/**
 * @brief exploration mode with Nav2 integration
 */
class AutonomousExplorerNav2 : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    AutonomousExplorerNav2() : Node("autonomous_explorer_nav2"), tf_broadcaster_(this) {
        initializeParameters();
        
        initializeSubscribers();
        initializePublishers();
        
        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");
        
        RCLCPP_INFO(this->get_logger(), "wait for init");
        startup_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "done init");
                exploration_timer_ = create_wall_timer(
                    std::chrono::milliseconds(static_cast<int>(1000.0 / exploration_frequency_)), 
                    std::bind(&AutonomousExplorerNav2::explorationCallback, this));
                startup_timer_->cancel();
            });
        
        visualization_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / visualization_frequency_)), 
            std::bind(&AutonomousExplorerNav2::visualizationCallback, this));
        
        current_state_ = ExplorationState::INITIALIZING;
        initial_orientation_set_ = false;
        
        RCLCPP_INFO(this->get_logger(), "INITIALIZING");
        RCLCPP_INFO(this->get_logger(), "using Nav2");
    }

private:
    void initializeParameters() {
        declare_parameter("map_topic", "/pointlio/cloud_registered");
        declare_parameter("pointcloud_topic", "/unilidar/cloud");
        declare_parameter("odom_topic", "/pointlio/odom");
        declare_parameter("cmd_vel_topic", "/cmd_vel");
        declare_parameter("global_frame", "camera_init");
        declare_parameter("robot_frame", "base_link");
        declare_parameter("exploration_frequency", 2.0);  // Hz
        declare_parameter("visualization_frequency", 5.0);  // Hz
        declare_parameter("min_frontier_size", 10);
        declare_parameter("frontier_detection_range", 5.0);  // meters probably
        declare_parameter("goal_tolerance", 0.5);  // meters probably
        declare_parameter("potential_scale", 5.0);
        declare_parameter("gain_scale", 1.0);
        declare_parameter("obstacle_cost_scale", 0.5);
        declare_parameter("rotation_threshold", 0.9);  // cos of angle
        declare_parameter("max_goal_distance", 5.0);  // meters
        declare_parameter("min_goal_distance", 0.5);  // meters
        declare_parameter("recovery_behaviors", std::vector<std::string>{"clear_costmap"});
        
        map_topic_ = get_parameter("map_topic").as_string();
        pointcloud_topic_ = get_parameter("pointcloud_topic").as_string();
        odom_topic_ = get_parameter("odom_topic").as_string();
        cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
        global_frame_ = get_parameter("global_frame").as_string();
        robot_frame_ = get_parameter("robot_frame").as_string();
        exploration_frequency_ = get_parameter("exploration_frequency").as_double();
        visualization_frequency_ = get_parameter("visualization_frequency").as_double();
        min_frontier_size_ = get_parameter("min_frontier_size").as_int();
        frontier_detection_range_ = get_parameter("frontier_detection_range").as_double();
        goal_tolerance_ = get_parameter("goal_tolerance").as_double();
        potential_scale_ = get_parameter("potential_scale").as_double();
        gain_scale_ = get_parameter("gain_scale").as_double();
        obstacle_cost_scale_ = get_parameter("obstacle_cost_scale").as_double();
        rotation_threshold_ = get_parameter("rotation_threshold").as_double();
        max_goal_distance_ = get_parameter("max_goal_distance").as_double();
        min_goal_distance_ = get_parameter("min_goal_distance").as_double();
        recovery_behaviors_ = get_parameter("recovery_behaviors").as_string_array();
    }
    
    void initializeSubscribers() {
        // sub to point cloud map from point-lio
        map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic_, 10, std::bind(&AutonomousExplorerNav2::mapCallback, this, std::placeholders::_1));
        
        // sub to cloud from lidar
        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 10, std::bind(&AutonomousExplorerNav2::pointCloudCallback, this, std::placeholders::_1));
        
        // sub to odo
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&AutonomousExplorerNav2::odometryCallback, this, std::placeholders::_1));
    }
    
    void initializePublishers() {
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/exploration/visualization_markers", 10);
        debug_pub_ = create_publisher<std_msgs::msg::String>("/exploration/debug", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/exploration/path", 10);
    }
    
    // debug statement, callback for point cloud updates
    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received point cloud map update - Size: %u x %u, Point step: %u, Row step: %u, Frame ID: %s",
                   msg->width, msg->height, msg->point_step, msg->row_step, msg->header.frame_id.c_str());
                   
        // just prints out the structure, just for debugging connection
        RCLCPP_DEBUG(this->get_logger(), "Point cloud fields:");
        for (size_t i = 0; i < msg->fields.size() && i < 5; ++i) {
            RCLCPP_DEBUG(this->get_logger(), "  Field %zu: %s (offset: %u, datatype: %u, count: %u)",
                       i, msg->fields[i].name.c_str(), msg->fields[i].offset, msg->fields[i].datatype, msg->fields[i].count);
        }
        
        // point cloud to occupancy grid projects the points onto a 2D grid

        current_map_.header = msg->header;
        current_map_.info.resolution = 0.1;
        current_map_.info.width = 200;
        current_map_.info.height = 200;
        current_map_.info.origin.position.x = -10.0;
        current_map_.info.origin.position.y = -10.0;
        current_map_.info.origin.position.z = 0.0;
        current_map_.info.origin.orientation.w = 1.0;
        
        current_map_.data.resize(current_map_.info.width * current_map_.info.height, -1);
        
        const uint8_t* data = msg->data.data();
        
        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (size_t i = 0; i < msg->fields.size(); ++i) {
            if (msg->fields[i].name == "x") x_offset = msg->fields[i].offset;
            if (msg->fields[i].name == "y") y_offset = msg->fields[i].offset;
            if (msg->fields[i].name == "z") z_offset = msg->fields[i].offset;
        }
        
        if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
            RCLCPP_ERROR(this->get_logger(), "Point cloud does not contain x, y, z fields");
            return;
        }
        
        // use pos from odo
        double robot_x = 0.0, robot_y = 0.0;
        if (odom_received_) {
            robot_x = current_odom_.pose.pose.position.x;
            robot_y = current_odom_.pose.pose.position.y;
        }
        
        // pos to grid
        int center_x = static_cast<int>((robot_x - current_map_.info.origin.position.x) / current_map_.info.resolution);
        int center_y = static_cast<int>((robot_y - current_map_.info.origin.position.y) / current_map_.info.resolution);
        
        // mark small area around the robot as free
        for (int dx = -5; dx <= 5; dx++) {
            for (int dy = -5; dy <= 5; dy++) {
                int x = center_x + dx;
                int y = center_y + dy;
                if (x >= 0 && x < static_cast<int>(current_map_.info.width) && 
                    y >= 0 && y < static_cast<int>(current_map_.info.height)) {
                    int index = y * current_map_.info.width + x;
                    current_map_.data[index] = 0;
                }
            }
        }
        
        // process point in point cloud
        for (uint32_t i = 0; i < msg->width * msg->height; ++i) {
            float x = *reinterpret_cast<const float*>(data + i * msg->point_step + x_offset);
            float y = *reinterpret_cast<const float*>(data + i * msg->point_step + y_offset);
            float z = *reinterpret_cast<const float*>(data + i * msg->point_step + z_offset);
            
            // skip points on z (boundary for vertical obstacles)
            if (z < -0.5 || z > 2.0) continue;
            
            int grid_x = static_cast<int>((x - current_map_.info.origin.position.x) / current_map_.info.resolution);
            int grid_y = static_cast<int>((y - current_map_.info.origin.position.y) / current_map_.info.resolution);
            
            if (grid_x >= 0 && grid_x < static_cast<int>(current_map_.info.width) && 
                grid_y >= 0 && grid_y < static_cast<int>(current_map_.info.height)) {
                
                int index = grid_y * current_map_.info.width + grid_x;
                current_map_.data[index] = 100;
                
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        if (dx == 0 && dy == 0) continue;
                        
                        int nx = grid_x + dx;
                        int ny = grid_y + dy;
                        
                        if (nx >= 0 && nx < static_cast<int>(current_map_.info.width) && 
                            ny >= 0 && ny < static_cast<int>(current_map_.info.height)) {
                            
                            int neighbor_index = ny * current_map_.info.width + nx;
                            if (current_map_.data[neighbor_index] == -1) {
                                current_map_.data[neighbor_index] = 0;
                            }
                        }
                    }
                }
            }
        }
        
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_received_ = true;
        
        if (current_state_ == ExplorationState::INITIALIZING) {
            current_state_ = ExplorationState::EXPLORING;
        }
    }
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "received pcd");
    }
    
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_odom_ = *msg;
        odom_received_ = true;
        
        // initial odo
        if (!initial_orientation_set_ && odom_received_) {
            initial_orientation_ = msg->pose.pose.orientation;
            initial_orientation_set_ = true;
            RCLCPP_INFO(this->get_logger(), "initial odo considered forward direction");
        }
        
        //broadcase from baselink to camera_init based on odo
        geometry_msgs::msg::TransformStamped transform;
        transform.header = msg->header;
        transform.header.frame_id = "camera_init";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;
        
        tf_broadcaster_.sendTransform(transform);
    }
    
    void explorationCallback() {
        if (!map_received_ || !odom_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "waiting for map and odo data...");
            return;
        }
        
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose)) {
            RCLCPP_WARN(this->get_logger(), "could not get robot pose, something broken");
            return;
        }
        
        switch (current_state_) {
            case ExplorationState::INITIALIZING:
                break;
                
            case ExplorationState::EXPLORING:
                exploreState(robot_pose);
                break;
                
            case ExplorationState::NAVIGATING_TO_FRONTIER:
                navigateState(robot_pose);
                break;
                
            case ExplorationState::ROTATING_FOR_BETTER_VIEW:
                rotateState(robot_pose);
                break;
                
            case ExplorationState::BACKTRACKING:
                backtrackState(robot_pose);
                break;
                
            case ExplorationState::RECOVERY:
                recoveryState(robot_pose);
                break;
                
            case ExplorationState::COMPLETED:
                completedState(robot_pose);
                break;
        }
    }
    
    // EXPLORING, detect frontiers and select the best
    void exploreState(const geometry_msgs::msg::PoseStamped& robot_pose) {
        RCLCPP_INFO(this->get_logger(), "Exploring state: detecting frontiers");
        
        std::vector<Frontier> frontiers = detectFrontiers();
        
        if (frontiers.empty()) {
            RCLCPP_INFO(this->get_logger(), "no frontiers, can transition to completed state");
            current_state_ = ExplorationState::COMPLETED;
            return;
        }
        
        current_frontier_ = selectBestFrontier(frontiers, robot_pose);
        
        if (current_frontier_.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "something with the frontier selection got messed up, recovery state");
            current_state_ = ExplorationState::RECOVERY;
            return;
        }
        
        // goal to frontier centroid
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = global_frame_;
        goal.header.stamp = this->now();
        goal.pose.position.x = current_frontier_.world_centroid.x;
        goal.pose.position.y = current_frontier_.world_centroid.y;
        goal.pose.position.z = 0.0;
        
        // direction to the goal relative to the robot's current position
        double dx = goal.pose.position.x - robot_pose.pose.position.x;
        double dy = goal.pose.position.y - robot_pose.pose.position.y;
        
        tf2::Quaternion q_current;
        tf2::fromMsg(robot_pose.pose.orientation, q_current);
        double current_yaw = tf2::getYaw(q_current);
        
        double initial_yaw = 0.0;
        if (initial_orientation_set_) {
            tf2::Quaternion q_initial;
            tf2::fromMsg(initial_orientation_, q_initial);
            initial_yaw = tf2::getYaw(q_initial);
        }
        
        double goal_direction = std::atan2(dy, dx);
        double forward_direction = current_yaw;
        double angle_diff = angles::shortest_angular_distance(forward_direction, goal_direction);
        
        double blend_factor = 0.3;
        double goal_yaw;
        
        if (std::abs(angle_diff) < M_PI_2) {
            goal_yaw = forward_direction * (1.0 - blend_factor) + goal_direction * blend_factor;
            RCLCPP_INFO(this->get_logger(), "Goal in front: forward_yaw=%.2f, goal_direction=%.2f, blended_yaw=%.2f",
                       forward_direction, goal_direction, goal_yaw);
        } else {
            goal_yaw = forward_direction * 0.7 + goal_direction * 0.3;
            RCLCPP_INFO(this->get_logger(), "Goal behind: forward_yaw=%.2f, goal_direction=%.2f, adjusted_yaw=%.2f",
                       forward_direction, goal_direction, goal_yaw);
        }
        
        tf2::Quaternion q;
        q.setRPY(0, 0, goal_yaw);
        goal.pose.orientation = tf2::toMsg(q);
        
        current_goal_ = goal;
        
        sendGoalToNav2(goal);
        
        current_state_ = ExplorationState::NAVIGATING_TO_FRONTIER;
        navigation_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "goal at [%f, %f], transitioning to: NAVIGATING_TO_FRONTIER",
                   goal.pose.position.x, goal.pose.position.y);
    }
    
    void navigateState(const geometry_msgs::msg::PoseStamped& robot_pose) {
        double distance_to_goal = std::sqrt(
            std::pow(robot_pose.pose.position.x - current_goal_.pose.position.x, 2) +
            std::pow(robot_pose.pose.position.y - current_goal_.pose.position.y, 2));
        
        if (distance_to_goal < goal_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "reached goal, go back to: EXPLORING");
            current_state_ = ExplorationState::EXPLORING;
            return;
        }
        
        auto now = this->now();
        auto navigation_duration = now - navigation_start_time_;
        if (navigation_duration > std::chrono::seconds(30)) {
            RCLCPP_WARN(this->get_logger(), "navigation timeout go back to EXPLORING");
            current_state_ = ExplorationState::EXPLORING;
            return;
        }
        
        if (last_robot_pose_.has_value() && navigation_duration > std::chrono::seconds(20)) {
            double movement = std::sqrt(
                std::pow(robot_pose.pose.position.x - last_robot_pose_->pose.position.x, 2) +
                std::pow(robot_pose.pose.position.y - last_robot_pose_->pose.position.y, 2));
                
            if (movement < 0.05) {
                RCLCPP_WARN(this->get_logger(), "trying a new frontier, robot not moving for 30 seconds, is it stuck?");
                current_state_ = ExplorationState::EXPLORING;
                return;
            }
            
            last_robot_pose_ = robot_pose;
        }
        
        last_robot_pose_ = robot_pose;
    }
    
    void rotateState(const geometry_msgs::msg::PoseStamped& robot_pose) {
        // ignore this block, just goes back to exploration
        RCLCPP_INFO(this->get_logger(), "entered dead function transitioning to state: EXPLORING");
        current_state_ = ExplorationState::EXPLORING;
    }
    
    // BACKTRACKING state: go back to a previous position
    void backtrackState(const geometry_msgs::msg::PoseStamped& robot_pose) {
        if (!backtrack_path_.empty()) {
            geometry_msgs::msg::PoseStamped waypoint = backtrack_path_.back();
            
            double distance_to_waypoint = std::sqrt(
                std::pow(robot_pose.pose.position.x - waypoint.pose.position.x, 2) +
                std::pow(robot_pose.pose.position.y - waypoint.pose.position.y, 2));
                
            if (distance_to_waypoint < goal_tolerance_) {
                backtrack_path_.pop_back();
                
                if (backtrack_path_.empty()) {
                    RCLCPP_INFO(this->get_logger(), "finish backtracking and going to state: EXPLORING");
                    current_state_ = ExplorationState::EXPLORING;
                    return;
                }
            }
            
            waypoint.header.stamp = this->now();
            sendGoalToNav2(waypoint);
        } else {
            RCLCPP_INFO(this->get_logger(), "cant backtrack and going to state: EXPLORING");
            current_state_ = ExplorationState::EXPLORING;
        }
    }
    
    void recoveryState(const geometry_msgs::msg::PoseStamped& robot_pose) {
        if (current_recovery_behavior_ == -1) {
            current_recovery_behavior_ = 0;
            recovery_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "starting recovery, found frontiers but none are valid or path cannot be planned");
            RCLCPP_INFO(this->get_logger(), "recovery behaviors available: %zu", recovery_behaviors_.size());
            for (size_t i = 0; i < recovery_behaviors_.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "  Behavior %zu: %s", i, recovery_behaviors_[i].c_str());
            }
        }
        
        if (current_recovery_behavior_ >= static_cast<int>(recovery_behaviors_.size())) {
            RCLCPP_WARN(this->get_logger(), "recovery failed, transitioning to: BACKTRACKING");
            current_recovery_behavior_ = -1;
            current_state_ = ExplorationState::BACKTRACKING;
            return;
        }
        
        std::string behavior = recovery_behaviors_[current_recovery_behavior_];
        
        auto now = this->now();
        auto recovery_duration = now - recovery_start_time_;
        
        if (recovery_duration > std::chrono::seconds(5)) {
            RCLCPP_INFO(this->get_logger(), "recovery '%s' completed after %d seconds", 
                       behavior.c_str(), 5);
            current_recovery_behavior_++;
            recovery_start_time_ = now;
            
            if (current_recovery_behavior_ >= static_cast<int>(recovery_behaviors_.size())) {
                RCLCPP_WARN(this->get_logger(), "recovery failed, transitioning to: BACKTRACKING");
                current_recovery_behavior_ = -1;
                current_state_ = ExplorationState::BACKTRACKING;
                return;
            }
            
            behavior = recovery_behaviors_[current_recovery_behavior_];
            RCLCPP_INFO(this->get_logger(), "Trying recovery behavior: %s (%d of %zu)", 
                       behavior.c_str(), current_recovery_behavior_ + 1, recovery_behaviors_.size());
        }
        
        if (behavior == "clear_costmap") {
            RCLCPP_INFO(this->get_logger(), "Recovery: clear costmap remove stale obstacle data");
            std::vector<Frontier> frontiers = detectFrontiers();
            RCLCPP_INFO(this->get_logger(), "cleared costmap, detected %zu frontiers", frontiers.size());
        }
        
        std_msgs::msg::String debug_msg;
        debug_msg.data = "Recovery state: Using behavior '" + behavior + "' (" + 
                         std::to_string(current_recovery_behavior_ + 1) + " of " + 
                         std::to_string(recovery_behaviors_.size()) + "), " +
                         std::to_string(5 - recovery_duration.seconds()) + " seconds remaining";
        debug_pub_->publish(debug_msg);
    }
    
    void completedState(const geometry_msgs::msg::PoseStamped& robot_pose) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, 
                            "no more frontiers exploration complete");
    }
    
    std::vector<Frontier> detectFrontiers() {
        std::vector<Frontier> frontiers;
        
        std::lock_guard<std::mutex> lock(map_mutex_);
        
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose)) {
            RCLCPP_WARN(this->get_logger(), "cant get robot pose for frontier detection");
            return frontiers;
        }
        
        int robot_grid_x = static_cast<int>((robot_pose.pose.position.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
        int robot_grid_y = static_cast<int>((robot_pose.pose.position.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
        
        int detection_range = static_cast<int>(frontier_detection_range_ / current_map_.info.resolution);
        
        std::vector<std::vector<bool>> visited(current_map_.info.height, std::vector<bool>(current_map_.info.width, false));
        
        for (int y = std::max(0, robot_grid_y - detection_range); 
             y < std::min(static_cast<int>(current_map_.info.height), robot_grid_y + detection_range); 
             y++) {
            for (int x = std::max(0, robot_grid_x - detection_range); 
                 x < std::min(static_cast<int>(current_map_.info.width), robot_grid_x + detection_range); 
                 x++) {
                
                if (visited[y][x]) continue;
                
                if (isFree(x, y) && hasUnknownNeighbor(x, y)) {
                    Frontier frontier;
                    
                    std::queue<GridPoint> queue;
                    queue.push(GridPoint(x, y));
                    visited[y][x] = true;
                    
                    while (!queue.empty()) {
                        GridPoint current = queue.front();
                        queue.pop();
                        
                        frontier.points.push_back(current);
                        
                        for (int dx = -1; dx <= 1; dx++) {
                            for (int dy = -1; dy <= 1; dy++) {
                                if (dx == 0 && dy == 0) continue;
                                
                                int nx = current.x + dx;
                                int ny = current.y + dy;
                                
                                if (nx < 0 || nx >= static_cast<int>(current_map_.info.width) || 
                                    ny < 0 || ny >= static_cast<int>(current_map_.info.height)) {
                                    continue;
                                }
                                
                                if (!visited[ny][nx] && isFree(nx, ny) && hasUnknownNeighbor(nx, ny)) {
                                    queue.push(GridPoint(nx, ny));
                                    visited[ny][nx] = true;
                                }
                            }
                        }
                    }
                    
                    if (frontier.points.size() >= static_cast<size_t>(min_frontier_size_)) {
                        frontier.calculateCentroid();
                        
                        frontier.world_centroid.x = frontier.centroid.x * current_map_.info.resolution + current_map_.info.origin.position.x;
                        frontier.world_centroid.y = frontier.centroid.y * current_map_.info.resolution + current_map_.info.origin.position.y;
                        
                        frontiers.push_back(frontier);
                    }
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Detected %zu frontiers", frontiers.size());
        return frontiers;
    }
    
    //utility calculation for best frontier selection
    Frontier selectBestFrontier(const std::vector<Frontier>& frontiers, const geometry_msgs::msg::PoseStamped& robot_pose) {
        if (frontiers.empty()) {
            return Frontier();
        }
        
        // calc  utility for each frontier
        std::vector<Frontier> scored_frontiers = frontiers;
        int valid_frontiers = 0;
        int too_far = 0;
        int too_close = 0;
        
        for (auto& frontier : scored_frontiers) {
            // calc distance to frontier
            double distance = std::sqrt(
                std::pow(robot_pose.pose.position.x - frontier.world_centroid.x, 2) +
                std::pow(robot_pose.pose.position.y - frontier.world_centroid.y, 2));
                
            // max distance threshold for frontier
            if (distance > max_goal_distance_) {
                frontier.utility = -1.0;
                too_far++;
                continue;
            }
            
            // min thresh
            if (distance < min_goal_distance_) {
                frontier.utility = -1.0;
                too_close++;
                continue;
            }
            
            valid_frontiers++;
            
            double information_gain = frontier.size;
            double cost = distance;
            
            frontier.utility = (gain_scale_ * information_gain) / (potential_scale_ * cost);
            
            tf2::Quaternion q_current;
            tf2::fromMsg(robot_pose.pose.orientation, q_current);
            double current_yaw = tf2::getYaw(q_current);
            
            double frontier_dx = frontier.world_centroid.x - robot_pose.pose.position.x;
            double frontier_dy = frontier.world_centroid.y - robot_pose.pose.position.y;
            double frontier_angle = std::atan2(frontier_dy, frontier_dx);
            
            double angle_diff = std::abs(angles::shortest_angular_distance(current_yaw, frontier_angle));
            
            //penalize frontiers that are behind the robot
            const double MAX_ANGLE = 75.0 * M_PI / 180.0;
            if (angle_diff > MAX_ANGLE) {
                double penalty = 1.0 - ((angle_diff - MAX_ANGLE) / (M_PI - MAX_ANGLE));
                frontier.utility *= std::max(0.1, penalty * penalty);
            }
            
            // hysteresis to favor the previous frontier, so that we can go in a consistent direction
            if (current_frontier_.points.size() > 0) {
                double dist_to_prev = std::sqrt(
                    std::pow(frontier.world_centroid.x - current_frontier_.world_centroid.x, 2) +
                    std::pow(frontier.world_centroid.y - current_frontier_.world_centroid.y, 2));
                    
                if (dist_to_prev < 2.0) {
                    frontier.utility *= 1.5;
                }
            }
        }
        
        Frontier best_frontier;
        double best_utility = -1.0;
        
        for (const auto& frontier : scored_frontiers) {
            if (frontier.utility > best_utility) {
                best_utility = frontier.utility;
                best_frontier = frontier;
            }
        }
        
        if (best_utility < 0.0) {
            RCLCPP_WARN(this->get_logger(), "No valid frontiers found (total: %zu, valid: %d, too far: %d, too close: %d)",
                       frontiers.size(), valid_frontiers, too_far, too_close);
            
            if (!frontiers.empty()) {
                double closest_dist = std::numeric_limits<double>::max();
                size_t closest_idx = 0;
                
                for (size_t i = 0; i < frontiers.size(); i++) {
                    double dist = std::sqrt(
                        std::pow(robot_pose.pose.position.x - frontiers[i].world_centroid.x, 2) +
                        std::pow(robot_pose.pose.position.y - frontiers[i].world_centroid.y, 2));
                    
                    if (dist < closest_dist) {
                        closest_dist = dist;
                        closest_idx = i;
                    }
                }
                
                RCLCPP_INFO(this->get_logger(), "using closest frontier at distance %f", closest_dist);
                return frontiers[closest_idx];
            }
            
            return Frontier();
        }
        
        RCLCPP_INFO(this->get_logger(), "selected a frontier with utility: %f, size: %f", 
                   best_utility, best_frontier.size);
                   
        return best_frontier;
    }
    
    void visualizationCallback() {
        if (!map_received_ || !odom_received_) {
            return;
        }
        
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose)) {
            return;
        }
        
        visualization_msgs::msg::MarkerArray marker_array;
        
        visualization_msgs::msg::Marker state_marker;
        state_marker.header.frame_id = global_frame_;
        state_marker.header.stamp = this->now();
        state_marker.ns = "exploration_state";
        state_marker.id = 0;
        state_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        state_marker.action = visualization_msgs::msg::Marker::ADD;
        state_marker.pose.position.x = robot_pose.pose.position.x;
        state_marker.pose.position.y = robot_pose.pose.position.y;
        state_marker.pose.position.z = 1.0;
        state_marker.pose.orientation.w = 1.0;
        state_marker.scale.z = 0.5;
        state_marker.color.r = 1.0;
        state_marker.color.g = 1.0;
        state_marker.color.b = 1.0;
        state_marker.color.a = 1.0;
        
        switch (current_state_) {
            case ExplorationState::INITIALIZING:
                state_marker.text = "INITIALIZING";
                break;
            case ExplorationState::EXPLORING:
                state_marker.text = "EXPLORING";
                break;
            case ExplorationState::NAVIGATING_TO_FRONTIER:
                state_marker.text = "NAVIGATING";
                break;
            case ExplorationState::ROTATING_FOR_BETTER_VIEW:
                state_marker.text = "ROTATING";
                break;
            case ExplorationState::BACKTRACKING:
                state_marker.text = "BACKTRACKING";
                break;
            case ExplorationState::RECOVERY:
                state_marker.text = "RECOVERY";
                break;
            case ExplorationState::COMPLETED:
                state_marker.text = "COMPLETED";
                break;
        }
        
        marker_array.markers.push_back(state_marker);
        
        if (current_state_ == ExplorationState::NAVIGATING_TO_FRONTIER) {
            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = global_frame_;
            goal_marker.header.stamp = this->now();
            goal_marker.ns = "exploration_goal";
            goal_marker.id = 0;
            goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            goal_marker.pose.position.x = current_goal_.pose.position.x;
            goal_marker.pose.position.y = current_goal_.pose.position.y;
            goal_marker.pose.position.z = 0.1;
            goal_marker.pose.orientation.w = 1.0;
            goal_marker.scale.x = 0.5;
            goal_marker.scale.y = 0.5;
            goal_marker.scale.z = 0.5;
            goal_marker.color.r = 0.0;
            goal_marker.color.g = 1.0;
            goal_marker.color.b = 0.0;
            goal_marker.color.a = 1.0;
            
            marker_array.markers.push_back(goal_marker);
            
            visualization_msgs::msg::Marker direction_marker;
            direction_marker.header.frame_id = global_frame_;
            direction_marker.header.stamp = this->now();
            direction_marker.ns = "exploration_direction";
            direction_marker.id = 0;
            direction_marker.type = visualization_msgs::msg::Marker::ARROW;
            direction_marker.action = visualization_msgs::msg::Marker::ADD;
            direction_marker.pose.position.x = robot_pose.pose.position.x;
            direction_marker.pose.position.y = robot_pose.pose.position.y;
            direction_marker.pose.position.z = 0.1;
            
            double dx = current_goal_.pose.position.x - robot_pose.pose.position.x;
            double dy = current_goal_.pose.position.y - robot_pose.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            
            if (distance > 0.001) {
                dx /= distance;
                dy /= distance;
                
                double yaw = std::atan2(dy, dx);
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                direction_marker.pose.orientation = tf2::toMsg(q);
            } else {
                direction_marker.pose.orientation.w = 1.0;
            }
            
            direction_marker.scale.x = 1.0;
            direction_marker.scale.y = 0.1; 
            direction_marker.scale.z = 0.1;
            
            direction_marker.color.r = 0.0;
            direction_marker.color.g = 1.0;
            direction_marker.color.b = 0.0;
            direction_marker.color.a = 1.0;
            
            marker_array.markers.push_back(direction_marker);
        }
        
        if (current_state_ == ExplorationState::EXPLORING || 
            current_state_ == ExplorationState::NAVIGATING_TO_FRONTIER) {
            std::vector<Frontier> frontiers = detectFrontiers();
            
            for (size_t i = 0; i < frontiers.size(); i++) {
                visualization_msgs::msg::Marker frontier_marker;
                frontier_marker.header.frame_id = global_frame_;
                frontier_marker.header.stamp = this->now();
                frontier_marker.ns = "frontiers";
                frontier_marker.id = static_cast<int>(i);
                frontier_marker.type = visualization_msgs::msg::Marker::SPHERE;
                frontier_marker.action = visualization_msgs::msg::Marker::ADD;
                frontier_marker.pose.position.x = frontiers[i].world_centroid.x;
                frontier_marker.pose.position.y = frontiers[i].world_centroid.y;
                frontier_marker.pose.position.z = 0.1;
                frontier_marker.pose.orientation.w = 1.0;
                frontier_marker.scale.x = 0.3;
                frontier_marker.scale.y = 0.3;
                frontier_marker.scale.z = 0.3;
                
                if (frontiers[i].utility > 0.0) {
                    double max_utility = 0.0;
                    for (const auto& f : frontiers) {
                        if (f.utility > max_utility) {
                            max_utility = f.utility;
                        }
                    }
                    
                    double normalized_utility = frontiers[i].utility / max_utility;
                    
                    frontier_marker.color.r = 1.0 - normalized_utility;
                    frontier_marker.color.g = normalized_utility;
                    frontier_marker.color.b = 0.0;
                } else {
                    frontier_marker.color.r = 0.5;
                    frontier_marker.color.g = 0.5;
                    frontier_marker.color.b = 0.5;
                }
                
                frontier_marker.color.a = 0.7;
                marker_array.markers.push_back(frontier_marker);
            }
        }
        
        marker_pub_->publish(marker_array);
    }
    
    bool getRobotPose(geometry_msgs::msg::PoseStamped& pose) {
        if (odom_received_) {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            
            pose.header = current_odom_.header;
            pose.pose = current_odom_.pose.pose;
            
            //assuming odo is in global frame
            return true;
        }
        
        return false;
    }
    
    bool isFree(int x, int y) const {
        if (x < 0 || x >= static_cast<int>(current_map_.info.width) || 
            y < 0 || y >= static_cast<int>(current_map_.info.height)) {
            return false;
        }
        
        int index = y * current_map_.info.width + x;
        return current_map_.data[index] >= 0 && current_map_.data[index] < 70;
    }
    
    bool isOccupied(int x, int y) const {
        if (x < 0 || x >= static_cast<int>(current_map_.info.width) || 
            y < 0 || y >= static_cast<int>(current_map_.info.height)) {
            return false;
        }
        
        int index = y * current_map_.info.width + x;
        return current_map_.data[index] >= 50;
    }
    
    bool isUnknown(int x, int y) const {
        if (x < 0 || x >= static_cast<int>(current_map_.info.width) || 
            y < 0 || y >= static_cast<int>(current_map_.info.height)) {
            return false;
        }
        
        int index = y * current_map_.info.width + x;
        return current_map_.data[index] == -1;
    }
    
    bool hasUnknownNeighbor(int x, int y) const {
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                
                if (isUnknown(x + dx, y + dy)) {
                    return true;
                }
            }
        }
        return false;
    }
    
    // send goal
    void sendGoalToNav2(const geometry_msgs::msg::PoseStamped& goal) {
        // wait for action server
        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "No Nav2 Acxtion server");
            return;
        }
        
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal;
        
        RCLCPP_INFO(this->get_logger(), "Sending goal to Nav2: [%f, %f]", 
                   goal.pose.position.x, goal.pose.position.y);
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&AutonomousExplorerNav2::goalResponseCallback, this, std::placeholders::_1);
            
        send_goal_options.feedback_callback =
            std::bind(&AutonomousExplorerNav2::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
            
        send_goal_options.result_callback =
            std::bind(&AutonomousExplorerNav2::resultCallback, this, std::placeholders::_1);
            
        nav2_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    void goalResponseCallback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by Nav2 - likely invalid position or collision");
            if (current_state_ == ExplorationState::NAVIGATING_TO_FRONTIER) {
                RCLCPP_INFO(this->get_logger(), "Going back to EXPLORING state to find a new frontier");
                current_state_ = ExplorationState::EXPLORING;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2");
        }
    }
    
    void feedbackCallback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_DEBUG(this->get_logger(), "Nav2 distance remaining: %f", feedback->distance_remaining);
    }
    
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "at goal");
                if (current_state_ == ExplorationState::NAVIGATING_TO_FRONTIER) {
                    current_state_ = ExplorationState::EXPLORING;
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "goal abort - likely preemption, planning fail, or obstacle");
                
                if (current_state_ == ExplorationState::NAVIGATING_TO_FRONTIER) {
                    RCLCPP_INFO(this->get_logger(), "Going back to EXPLORING state to find a new frontier");
                    current_state_ = ExplorationState::EXPLORING;
                }
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "canceled");
                if (current_state_ == ExplorationState::NAVIGATING_TO_FRONTIER) {
                    RCLCPP_INFO(this->get_logger(), "Going back to EXPLORING state to find a new frontier");
                    current_state_ = ExplorationState::EXPLORING;
                }
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "unknown");
                if (current_state_ == ExplorationState::NAVIGATING_TO_FRONTIER) {
                    RCLCPP_INFO(this->get_logger(), "Going back to EXPLORING state to find a new frontier");
                    current_state_ = ExplorationState::EXPLORING;
                }
                break;
        }
    }
    
    std::string map_topic_;
    std::string pointcloud_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    std::string global_frame_;
    std::string robot_frame_;
    double exploration_frequency_;
    double visualization_frequency_;
    int min_frontier_size_;
    double frontier_detection_range_;
    double goal_tolerance_;
    double potential_scale_;
    double gain_scale_;
    double obstacle_cost_scale_;
    double rotation_threshold_;
    double max_goal_distance_;
    double min_goal_distance_;
    std::vector<std::string> recovery_behaviors_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr exploration_timer_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::mutex map_mutex_;
    std::mutex odom_mutex_;
    geometry_msgs::msg::Quaternion initial_orientation_;
    bool initial_orientation_set_;
    nav_msgs::msg::OccupancyGrid current_map_;
    nav_msgs::msg::Odometry current_odom_;
    bool map_received_ = false;
    bool odom_received_ = false;
    
    ExplorationState current_state_ = ExplorationState::INITIALIZING;
    Frontier current_frontier_;
    geometry_msgs::msg::PoseStamped current_goal_;
    std::optional<geometry_msgs::msg::PoseStamped> last_robot_pose_;
    std::vector<geometry_msgs::msg::PoseStamped> backtrack_path_;
    
    rclcpp::Time navigation_start_time_;
    rclcpp::Time rotation_start_time_;
    rclcpp::Time recovery_start_time_;
    
    int current_recovery_behavior_ = -1;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousExplorerNav2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
