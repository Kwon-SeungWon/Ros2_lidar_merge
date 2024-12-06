#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class MotionController : public rclcpp::Node {
public:
    MotionController() : Node("motion_controller") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&MotionController::executeMotion, this));

        // Clock 객체 생성
        clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);  // System time
    }

private:
    void executeMotion() {
        if (motion_state_ == 0) {
            moveStraight(2.0);  // Move forward 2 meters
        } else if (motion_state_ == 1) {
            rotate(45.0);  // Rotate 45 degrees
        } else if (motion_state_ == 2) {
            moveStraight(2.0);  // Move forward another 2 meters
        } else {
            stop();  // Stop the robot
        }
    }

    void moveStraight(double distance) {
        auto current_time = clock_->now();  // Clock
        if (start_time_ == rclcpp::Time(0, 0, RCL_SYSTEM_TIME)) {  // If start_time_ is not set
            start_time_ = current_time;
        }
        double duration = distance / linear_speed_;
        if ((current_time - start_time_).seconds() < duration) {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = linear_speed_;
            msg.angular.z = 0.0;
            publisher_->publish(msg);
        } else {
            motion_state_++;
            start_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);  // Reset start_time_
        }
    }

    void rotate(double angle) {
        auto current_time = clock_->now();  // Clock
        if (start_time_ == rclcpp::Time(0, 0, RCL_SYSTEM_TIME)) {  // If start_time_ is not set
            start_time_ = current_time;
        }
        double duration = angle * M_PI / 180.0 / angular_speed_;  // Convert angle to radians
        if ((current_time - start_time_).seconds() < duration) {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.0;
            msg.angular.z = -angular_speed_;
            publisher_->publish(msg);
        } else {
            motion_state_++;
            start_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);  // Reset start_time_
        }
    }

    void stop() {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Clock> clock_;  // Clock

    int motion_state_ = 0;  // Tracks the current motion state
    rclcpp::Time start_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);  // Start time with specific time source
    const double linear_speed_ = 0.2;  // m/s
    const double angular_speed_ = M_PI / 4.0;  // rad/s (45 degrees in radians)
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionController>());
    rclcpp::shutdown();
    return 0;
}
