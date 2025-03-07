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

// Im not entire sure how well these work
#include "../include/dubins_path.hpp"
#include "../include/cubic_spline.hpp"

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
 * @brief exploration mode
 */
class AutonomousExplorer : public rclcpp::Node {
public:
    AutonomousExplorer() : Node("autonomous_explorer") {
        initializeParameters();
        
        // using odometry for now but i believe TF is better use
        // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        initializeSubscribers();
        initializePublishers();
        
        exploration_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / exploration_frequency_)), 
            std::bind(&AutonomousExplorer::explorationCallback, this));
        
        visualization_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / visualization_frequency_)), 
            std::bind(&AutonomousExplorer::visualizationCallback, this));
        
        current_state_ = ExplorationState::INITIALIZING;
        
        RCLCPP_INFO(this->get_logger(), "Robot in state: INITIALIZING");
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
        declare_parameter("max_goal_distance", 10.0);  // meters
        declare_parameter("min_goal_distance", 1.0);  // meters
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
            map_topic_, 10, std::bind(&AutonomousExplorer::mapCallback, this, std::placeholders::_1));
        
        // sub to cloud from lidar
        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 10, std::bind(&AutonomousExplorer::pointCloudCallback, this, std::placeholders::_1));
        
        // sub to odo
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&AutonomousExplorer::odometryCallback, this, std::placeholders::_1));
    }
    
    void initializePublishers() {
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/exploration/visualization_markers", 10);
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
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
        
        // transition to exploring
        if (current_state_ == ExplorationState::INITIALIZING) {
            current_state_ = ExplorationState::EXPLORING;
            RCLCPP_INFO(this->get_logger(), "getting maps, begin exploring: EXPLORING");
        }
    }
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "received pcd");
    }
    
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_odom_ = *msg;
        odom_received_ = true;
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
                
            // this is messed up, we probably dont need this state, and this is nto used
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
        
        double dx = goal.pose.position.x - robot_pose.pose.position.x;
        double dy = goal.pose.position.y - robot_pose.pose.position.y;
        double goal_yaw = std::atan2(dy, dx);
        
        tf2::Quaternion q;
        q.setRPY(0, 0, goal_yaw);
        goal.pose.orientation = tf2::toMsg(q);
        
        goal_pub_->publish(goal);
        current_goal_ = goal;
        
        // nagigation state, pathfinding time
        current_state_ = ExplorationState::NAVIGATING_TO_FRONTIER;
        navigation_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "goal at [%f, %f], transitioning to: NAVIGATING_TO_FRONTIER",
                   goal.pose.position.x, goal.pose.position.y);
    }
    
    // path visualization, not working for some reason
    void visualizePath(const std::vector<dubins::Pose2D>& path, const std_msgs::msg::Header& header) {
        if (path.empty()) {
            return;
        }
        
        nav_msgs::msg::Path path_msg;
        path_msg.header = header;
        path_msg.header.frame_id = global_frame_;
        
        for (const auto& pose : path) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.pose.position.x = pose.x;
            pose_stamped.pose.position.y = pose.y;
            pose_stamped.pose.position.z = 0.1; 
            
            tf2::Quaternion q;
            q.setRPY(0, 0, pose.theta);
            pose_stamped.pose.orientation = tf2::toMsg(q);
            
            path_msg.poses.push_back(pose_stamped);
        }
        
        path_pub_->publish(path_msg);
        
        visualization_msgs::msg::MarkerArray marker_array;
        
        visualization_msgs::msg::Marker line_marker;
        line_marker.header = path_msg.header;
        line_marker.ns = "exploration_path";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.1;
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;
        line_marker.lifetime = rclcpp::Duration(10, 0);
        
        visualization_msgs::msg::Marker points_marker;
        points_marker.header = path_msg.header;
        points_marker.ns = "exploration_path_points";
        points_marker.id = 0;
        points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::msg::Marker::ADD;
        points_marker.scale.x = 0.2;
        points_marker.scale.y = 0.2;
        points_marker.scale.z = 0.2;
        points_marker.color.r = 1.0;
        points_marker.color.g = 1.0;
        points_marker.color.b = 0.0;
        points_marker.color.a = 1.0;
        points_marker.lifetime = rclcpp::Duration(10, 0);
        
        for (const auto& pose : path) {
            geometry_msgs::msg::Point p;
            p.x = pose.x;
            p.y = pose.y;
            p.z = 0.1;
            
            line_marker.points.push_back(p);
            points_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_marker);
        marker_array.markers.push_back(points_marker);
        
        marker_pub_->publish(marker_array);
        
        //right now we're seeing paths in terminal but rviz is not displaying them for some reason
        RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path.size());
    }
    
    // NAVIGATING_TO_FRONTIER state: follow the path to goal
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
        //timeout param
        if (navigation_duration > std::chrono::seconds(60)) {
            RCLCPP_WARN(this->get_logger(), "navigation timeout, we try to explore again: EXPLORING");
            current_state_ = ExplorationState::EXPLORING;
            return;
        }
        
        // timeot param for if we're making progress
        if (last_robot_pose_.has_value() && navigation_duration > std::chrono::seconds(15)) {
            double movement = std::sqrt(
                std::pow(robot_pose.pose.position.x - last_robot_pose_->pose.position.x, 2) +
                std::pow(robot_pose.pose.position.y - last_robot_pose_->pose.position.y, 2));
                
            // consider robot as stuck if no movement detected for n seconds
            if (movement < 0.01) {
                RCLCPP_WARN(this->get_logger(), "trying a new frontier, robot not moving, is it stuck?");
                // right now im skipping recovery to go back to exploring
                current_state_ = ExplorationState::EXPLORING;
                return;
            }
        }
        
        last_robot_pose_ = robot_pose;
        
        if (!current_path_.empty() && current_waypoint_index_ < current_path_.size()) {
            dubins::Pose2D waypoint = current_path_[current_waypoint_index_];
            
            double distance_to_waypoint = std::sqrt(
                std::pow(robot_pose.pose.position.x - waypoint.x, 2) +
                std::pow(robot_pose.pose.position.y - waypoint.y, 2));
            
            if (distance_to_waypoint < waypoint_tolerance_) {
                current_waypoint_index_++;
                //check if we are getting to which waypoint from path
                RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu of %zu", 
                           current_waypoint_index_, current_path_.size());
                
                if (current_waypoint_index_ >= current_path_.size()) {
                    RCLCPP_INFO(this->get_logger(), "end of path but not at goal");
                    return;
                }
                
                waypoint = current_path_[current_waypoint_index_];
            }
            
            geometry_msgs::msg::PoseStamped goal;
            goal.header.frame_id = global_frame_;
            goal.header.stamp = this->now();
            goal.pose.position.x = waypoint.x;
            goal.pose.position.y = waypoint.y;
            goal.pose.position.z = 0.0;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, waypoint.theta);
            goal.pose.orientation = tf2::toMsg(q);
            
            goal_pub_->publish(goal);
            
            RCLCPP_DEBUG(this->get_logger(), "Following path: waypoint %zu/%zu at [%f, %f]", 
                        current_waypoint_index_ + 1, current_path_.size(), waypoint.x, waypoint.y);
        } else {
            // we really shouldnt need this, this is a hardcode for forcing it to place another waypoint at the goal
            // which is not good behavior if path planning is working
            goal_pub_->publish(current_goal_);
        }
    }
    
    void rotateState(const geometry_msgs::msg::PoseStamped& robot_pose) {
        // ignore this block, just goes b ack to exploration
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
            goal_pub_->publish(waypoint);
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
        
        // this doesnt work
        if (behavior == "rotate") {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.angular.z = 0.5;
            cmd_vel_pub_->publish(cmd_vel);
        //this is just getting logged, we should be clearing the costmap ourselves later
        } else if (behavior == "clear_costmap") {
            RCLCPP_INFO(this->get_logger(), "Recovery: clear costmap remove stale obstacle data");
                        std::vector<Frontier> frontiers = detectFrontiers();
            RCLCPP_INFO(this->get_logger(), "cleared costmap, detected %zu frontiers", frontiers.size());
        } else if (behavior == "backup") {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = -0.2;
            cmd_vel_pub_->publish(cmd_vel);
            
            if (recovery_duration.seconds() == 0 || static_cast<int>(recovery_duration.seconds()) % 2 == 0) {
                RCLCPP_INFO(this->get_logger(), "backing up to move away from obstacles");
            }
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
    
    //this is bugged idk why
    void visualizationCallback() {
        if (!map_received_ || !odom_received_) {
            return;
        }
        
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose)) {
            return;
        }
        
        visualization_msgs::msg::MarkerArray test_marker_array;
        visualization_msgs::msg::Marker test_path_marker;
        test_path_marker.header.frame_id = global_frame_;
        test_path_marker.header.stamp = this->now();
        test_path_marker.ns = "frontier_paths";
        test_path_marker.id = 999;
        test_path_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        test_path_marker.action = visualization_msgs::msg::Marker::ADD;
        test_path_marker.scale.x = 0.2;
        test_path_marker.scale.y = 0.2;
        test_path_marker.scale.z = 0.2;
        test_path_marker.color.r = 1.0;
        test_path_marker.color.g = 0.0;
        test_path_marker.color.b = 1.0;
        test_path_marker.color.a = 1.0;
        test_path_marker.lifetime = rclcpp::Duration(5, 0);
        
        for (double x = -2.0; x <= 2.0; x += 0.4) {
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = 1.0;
            p.z = 0.1;
            test_path_marker.points.push_back(p);
        }
        
        test_marker_array.markers.push_back(test_path_marker);
        marker_pub_->publish(test_marker_array);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "publishing a test marker in frontier_paths namespace");
        
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
                
                std::vector<dubins::Pose2D> path_to_frontier;
                dubins::Pose2D start_pose(
                    robot_pose.pose.position.x,
                    robot_pose.pose.position.y,
                    tf2::getYaw(robot_pose.pose.orientation)
                );
                dubins::Pose2D goal_pose(
                    frontiers[i].world_centroid.x,
                    frontiers[i].world_centroid.y,
                    std::atan2(frontiers[i].world_centroid.y - robot_pose.pose.position.y,
                              frontiers[i].world_centroid.x - robot_pose.pose.position.x)
                );
                
                //turn radisus
                path_planner_.set_turning_radius(0.5);
                dubins::Path dubins_path = path_planner_.plan(start_pose, goal_pose);
                path_to_frontier = path_planner_.generate_path_poses(start_pose, dubins_path, 0.1);
                
                RCLCPP_INFO(this->get_logger(), "Frontier %zu: Path planning from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
                           i, start_pose.x, start_pose.y, start_pose.theta,
                           goal_pose.x, goal_pose.y, goal_pose.theta);
                
                if (!path_to_frontier.empty()) {
                    RCLCPP_INFO(this->get_logger(), "  Path has %zu waypoints:", path_to_frontier.size());
                    if (path_to_frontier.size() > 0) {
                        RCLCPP_INFO(this->get_logger(), "    First: [%.2f, %.2f, %.2f]",
                                   path_to_frontier[0].x, path_to_frontier[0].y, path_to_frontier[0].theta);
                    }
                    if (path_to_frontier.size() > 2) {
                        size_t mid = path_to_frontier.size() / 2;
                        RCLCPP_INFO(this->get_logger(), "    Middle: [%.2f, %.2f, %.2f]",
                                   path_to_frontier[mid].x, path_to_frontier[mid].y, path_to_frontier[mid].theta);
                    }
                    if (path_to_frontier.size() > 1) {
                        RCLCPP_INFO(this->get_logger(), "    Last: [%.2f, %.2f, %.2f]",
                                   path_to_frontier.back().x, path_to_frontier.back().y, path_to_frontier.back().theta);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "  No valid path found to frontier %zu", i);
                }
                
                if (!path_to_frontier.empty() && frontiers[i].utility > 0.0) {
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
                    
                    visualization_msgs::msg::Marker path_marker;
                    path_marker.header.frame_id = global_frame_; 
                    path_marker.header.stamp = this->now();
                    path_marker.ns = "frontier_paths";
                    path_marker.id = static_cast<int>(i);
                    path_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
                    path_marker.action = visualization_msgs::msg::Marker::ADD;
                    path_marker.scale.x = 0.3;
                    path_marker.scale.y = 0.3;
                    path_marker.scale.z = 0.3;
                    path_marker.color.r = 1.0;
                    path_marker.color.g = 0.0;
                    path_marker.color.b = 0.0;
                    path_marker.color.a = 1.0;
                    path_marker.lifetime = rclcpp::Duration(10, 0);
                    
                    RCLCPP_INFO(this->get_logger(), "path marker for frontier %zu and %zu points", 
                               i, path_to_frontier.size());
                    
                    for (const auto& pose : path_to_frontier) {
                        geometry_msgs::msg::Point p;
                        p.x = pose.x;
                        p.y = pose.y;
                        p.z = 0.05;
                        path_marker.points.push_back(p);
                    }
                    
                    marker_array.markers.push_back(path_marker);
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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    rclcpp::TimerBase::SharedPtr exploration_timer_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::mutex map_mutex_;
    std::mutex odom_mutex_;
    nav_msgs::msg::OccupancyGrid current_map_;
    nav_msgs::msg::Odometry current_odom_;
    bool map_received_ = false;
    bool odom_received_ = false;
    
    ExplorationState current_state_ = ExplorationState::INITIALIZING;
    Frontier current_frontier_;
    geometry_msgs::msg::PoseStamped current_goal_;
    std::optional<geometry_msgs::msg::PoseStamped> last_robot_pose_;
    std::vector<geometry_msgs::msg::PoseStamped> backtrack_path_;
    
    dubins::DubinsPlanner path_planner_{0.2};//turning radius
    std::vector<dubins::Pose2D> current_path_;
    size_t current_waypoint_index_ = 0;
    double waypoint_tolerance_ = 0.3;
    
    rclcpp::Time navigation_start_time_;
    rclcpp::Time rotation_start_time_;
    rclcpp::Time recovery_start_time_;
    
    int current_recovery_behavior_ = -1;
    
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
