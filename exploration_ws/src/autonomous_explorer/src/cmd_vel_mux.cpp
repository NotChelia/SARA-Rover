#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <memory>
#include <string>

class CmdVelMux : public rclcpp::Node {
public:
    CmdVelMux() : Node("cmd_vel_mux") {
        declare_parameter("joystick_topic", "/joy");
        declare_parameter("explorer_cmd_vel_topic", "/cmd_vel_explorer");
        declare_parameter("output_cmd_vel_topic", "/cmd_vel");
        declare_parameter("joystick_threshold", 0.2);
        declare_parameter("timeout_duration", 0.5);
        
        joystick_topic_ = get_parameter("joystick_topic").as_string();
        explorer_cmd_vel_topic_ = get_parameter("explorer_cmd_vel_topic").as_string();
        output_cmd_vel_topic_ = get_parameter("output_cmd_vel_topic").as_string();
        joystick_threshold_ = get_parameter("joystick_threshold").as_double();
        timeout_duration_ = get_parameter("timeout_duration").as_double();
        
        joystick_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            joystick_topic_, 10, std::bind(&CmdVelMux::joystickCallback, this, std::placeholders::_1));
            
        explorer_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            explorer_cmd_vel_topic_, 10, std::bind(&CmdVelMux::explorerCmdVelCallback, this, std::placeholders::_1));
            
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
        
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CmdVelMux::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "CmdVelMux initialized");
        RCLCPP_INFO(this->get_logger(), "Joystick topic: %s", joystick_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Explorer cmd_vel topic: %s", explorer_cmd_vel_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output cmd_vel topic: %s", output_cmd_vel_topic_.c_str());
    }
    
private:
    void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        last_joystick_time_ = this->now();
        
        bool joystick_active = false;
        for (size_t i = 0; i < msg->axes.size() && i < 4; ++i) {
            if (std::abs(msg->axes[i]) > joystick_threshold_) {
                joystick_active = true;
                break;
            }
        }
        
        if (joystick_active) {
            geometry_msgs::msg::Twist cmd_vel;
            if (msg->axes.size() > 1) {
                cmd_vel.linear.x = msg->axes[1] * 0.2;
                cmd_vel.angular.z = msg->axes[0] * 1.24;
            }
            cmd_vel_pub_->publish(cmd_vel);
            
            active_source_ = Source::JOYSTICK;
            
            RCLCPP_INFO(this->get_logger(), "Joystick koko: linear.x=%f, angular.z=%f",
                       cmd_vel.linear.x, cmd_vel.angular.z);
        } else if (active_source_ == Source::JOYSTICK) {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;
            
            cmd_vel_pub_->publish(cmd_vel);
            
            active_source_ = Source::EXPLORER;
            
            RCLCPP_INFO(this->get_logger(), "joystick inai");
        }
    }
    
    void explorerCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_explorer_cmd_vel_time_ = this->now();
        last_explorer_cmd_vel_ = *msg;
        
        if (active_source_ == Source::EXPLORER) {
            cmd_vel_pub_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Explopre koko: linear.x=%f, angular.z=%f",
                       msg->linear.x, msg->angular.z);
        }
    }
    
    void timerCallback() {
        auto now = this->now();
        
        if (active_source_ == Source::JOYSTICK) {
            if (last_joystick_time_ != rclcpp::Time(0, 0, this->get_clock()->get_clock_type())) {
                double joystick_elapsed = (now - last_joystick_time_).seconds();
                if (joystick_elapsed > timeout_duration_) {
                    RCLCPP_INFO(this->get_logger(), "no joystick going to explorer");
                    active_source_ = Source::EXPLORER;
                    
                    if (last_explorer_cmd_vel_time_ != rclcpp::Time(0, 0, this->get_clock()->get_clock_type())) {
                        cmd_vel_pub_->publish(last_explorer_cmd_vel_);
                    }
                }
            }
        }
        
        if (active_source_ == Source::EXPLORER) {
            if (last_explorer_cmd_vel_time_ != rclcpp::Time(0, 0, this->get_clock()->get_clock_type())) {
                double explorer_elapsed = (now - last_explorer_cmd_vel_time_).seconds();
                if (explorer_elapsed > timeout_duration_) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                       "nothing coming out of explorer big problem",
                                       explorer_elapsed);
                    
                    geometry_msgs::msg::Twist cmd_vel;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.linear.y = 0.0;
                    cmd_vel.linear.z = 0.0;
                    cmd_vel.angular.x = 0.0;
                    cmd_vel.angular.y = 0.0;
                    cmd_vel.angular.z = 0.0;
                    
                    cmd_vel_pub_->publish(cmd_vel);
                }
            }
        }
    }
    
    enum class Source {
        JOYSTICK,
        EXPLORER
    };
    
    std::string joystick_topic_;
    std::string explorer_cmd_vel_topic_;
    std::string output_cmd_vel_topic_;
    double joystick_threshold_;
    double timeout_duration_;
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr explorer_cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    Source active_source_ = Source::EXPLORER;
    rclcpp::Time last_joystick_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_explorer_cmd_vel_time_{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::Twist last_explorer_cmd_vel_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelMux>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
