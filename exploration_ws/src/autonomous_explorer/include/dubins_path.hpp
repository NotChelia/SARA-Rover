#ifndef DUBINS_PATH_HPP
#define DUBINS_PATH_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>
#include <string>

namespace dubins {

enum class PathSegmentType {
    LEFT, RIGHT, STRAIGHT
};

struct PathSegment {
    PathSegmentType type;
    double length;
    
    PathSegment(PathSegmentType t, double l) : type(t), length(l) {}
};

struct Path {
    std::vector<PathSegment> segments;
    double total_length;
    
    Path() : total_length(std::numeric_limits<double>::max()) {}
    
    void clear() {
        segments.clear();
        total_length = std::numeric_limits<double>::max();
    }
    
    void add_segment(PathSegmentType type, double length) {
        segments.push_back(PathSegment(type, length));
        if (total_length == std::numeric_limits<double>::max()) {
            total_length = 0;
        }
        total_length += std::abs(length);
    }
    
    bool is_valid() const {
        return total_length < std::numeric_limits<double>::max();
    }
};

struct Pose2D {
    double x;
    double y;
    double theta;
    
    Pose2D() : x(0), y(0), theta(0) {}
    Pose2D(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}
    
    double distance_to(const Pose2D& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }
};

inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

class DubinsPlanner {
public:
    DubinsPlanner(double turning_radius = 1.0) 
        : turning_radius_(turning_radius) {}
    
    void set_turning_radius(double turning_radius) {
        turning_radius_ = turning_radius;
    }
    
    Path plan(const Pose2D& start, const Pose2D& goal) {
        double dx = goal.x - start.x;
        double dy = goal.y - start.y;
        double c = std::cos(start.theta);
        double s = std::sin(start.theta);
        double x = c * dx + s * dy;
        double y = -s * dx + c * dy;
        double phi = normalize_angle(goal.theta - start.theta);
        
        std::vector<Path> paths;
        
        compute_LSL_path(x, y, phi, paths);
        compute_RSR_path(x, y, phi, paths);
        compute_LSR_path(x, y, phi, paths);
        compute_RSL_path(x, y, phi, paths);
        
        Path shortest_path;
        double min_length = std::numeric_limits<double>::max();
        
        for (const auto& path : paths) {
            if (path.is_valid() && path.total_length < min_length) {
                min_length = path.total_length;
                shortest_path = path;
            }
        }
        
        return shortest_path;
    }
    
    std::vector<Pose2D> generate_path_poses(const Pose2D& start, const Path& path, double step_size = 0.1) {
        std::vector<Pose2D> poses;
        
        if (!path.is_valid()) {
            return poses;
        }
        
        poses.push_back(start);
        
        Pose2D current = start;
        
        for (const auto& segment : path.segments) {
            double length = segment.length;
            double abs_length = std::abs(length);
            int steps = std::max(1, static_cast<int>(abs_length / step_size));
            double step_length = length / steps;
            
            for (int i = 0; i < steps; ++i) {
                if (segment.type == PathSegmentType::STRAIGHT) {
                    current.x += step_length * std::cos(current.theta);
                    current.y += step_length * std::sin(current.theta);
                } else if (segment.type == PathSegmentType::LEFT) {
                    double phi = step_length / turning_radius_;
                    double r = turning_radius_;
                    double cx = current.x - r * std::sin(current.theta);
                    double cy = current.y + r * std::cos(current.theta);
                    current.theta = normalize_angle(current.theta + phi);
                    current.x = cx + r * std::sin(current.theta);
                    current.y = cy - r * std::cos(current.theta);
                } else if (segment.type == PathSegmentType::RIGHT) {
                    double phi = step_length / turning_radius_;
                    double r = turning_radius_;
                    double cx = current.x + r * std::sin(current.theta);
                    double cy = current.y - r * std::cos(current.theta);
                    current.theta = normalize_angle(current.theta - phi);
                    current.x = cx - r * std::sin(current.theta);
                    current.y = cy + r * std::cos(current.theta);
                }
                
                poses.push_back(current);
            }
        }
        
        return poses;
    }
    
private:
    double turning_radius_;
    
    void compute_LSL_path(double x, double y, double phi, std::vector<Path>& paths) {
        Path path;
        double r = turning_radius_;
        double u = x - r * std::sin(phi);
        double v = y + r * std::cos(phi) - r;
        double t = std::atan2(v, u);
        double d = std::sqrt(u*u + v*v);
        
        if (d >= 2 * r) {
            double alpha = std::asin(2 * r / d);
            double beta = t - alpha;
            double s = 2 * r * std::cos(alpha);
            
            path.add_segment(PathSegmentType::LEFT, r * beta);
            path.add_segment(PathSegmentType::STRAIGHT, s);
            path.add_segment(PathSegmentType::LEFT, r * (phi - beta));
            
            paths.push_back(path);
        }
    }
    
    void compute_RSR_path(double x, double y, double phi, std::vector<Path>& paths) {
        Path path;
        double r = turning_radius_;
        double u = x + r * std::sin(phi);
        double v = y - r * std::cos(phi) - r;
        double t = std::atan2(v, u);
        double d = std::sqrt(u*u + v*v);
        
        if (d >= 2 * r) {
            double alpha = std::asin(2 * r / d);
            double beta = t + alpha;
            double s = 2 * r * std::cos(alpha);
            
            path.add_segment(PathSegmentType::RIGHT, r * beta);
            path.add_segment(PathSegmentType::STRAIGHT, s);
            path.add_segment(PathSegmentType::RIGHT, r * (beta - phi));
            
            paths.push_back(path);
        }
    }
    
    void compute_LSR_path(double x, double y, double phi, std::vector<Path>& paths) {
        Path path;
        double r = turning_radius_;
        double u = x - r * std::sin(phi);
        double v = y + r * std::cos(phi) + r;
        double t = std::atan2(v, u);
        double d = std::sqrt(u*u + v*v);
        
        if (d >= 2 * r) {
            double alpha = std::acos(2 * r / d);
            double beta = t + alpha;
            double s = d * std::sin(alpha);
            
            path.add_segment(PathSegmentType::LEFT, r * beta);
            path.add_segment(PathSegmentType::STRAIGHT, s);
            path.add_segment(PathSegmentType::RIGHT, r * (beta + phi));
            
            paths.push_back(path);
        }
    }
    
    void compute_RSL_path(double x, double y, double phi, std::vector<Path>& paths) {
        Path path;
        double r = turning_radius_;
        double u = x + r * std::sin(phi);
        double v = y - r * std::cos(phi) + r;
        double t = std::atan2(v, u);
        double d = std::sqrt(u*u + v*v);
        
        if (d >= 2 * r) {
            double alpha = std::acos(2 * r / d);
            double beta = t - alpha;
            double s = d * std::sin(alpha);
            
            path.add_segment(PathSegmentType::RIGHT, r * beta);
            path.add_segment(PathSegmentType::STRAIGHT, s);
            path.add_segment(PathSegmentType::LEFT, r * (phi - beta));
            
            paths.push_back(path);
        }
    }
};

} // namespace dubins

#endif // DUBINS_PATH_HPP
