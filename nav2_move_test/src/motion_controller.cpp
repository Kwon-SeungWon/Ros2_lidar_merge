#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

class NavigateStraight : public rclcpp::Node {
public:
    NavigateStraight()
        : Node("navigate_straight"){
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Action Client 생성
        nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");
    }

    void start_navigation() {
        // Action 서버가 준비될 때까지 대기
        if (!nav_to_pose_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // 현재 위치 가져오기
        geometry_msgs::msg::PoseStamped current_pose;
        if (!get_current_pose(current_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current pose");
            return;
        }

        // 현재 방향(yaw) 계산
        tf2::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 목표 위치 계산 (현재 방향으로 2m 직진)
        geometry_msgs::msg::PoseStamped goal_pose = current_pose;
        goal_pose.pose.position.x += 2.0 * cos(yaw);
        goal_pose.pose.position.y += 2.0 * sin(yaw);

        RCLCPP_INFO(this->get_logger(), "Navigating to goal: x=%.2f, y=%.2f",
                    goal_pose.pose.position.x, goal_pose.pose.position.y);

        // NavigateToPose 요청 생성
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        // 요청 전송
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](auto result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Navigation failed!");
            }
        };

        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void start_navigation2() {
        // Action 서버가 준비될 때까지 대기
        if (!nav_to_pose_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // 현재 위치 가져오기
        geometry_msgs::msg::PoseStamped current_pose;
        if (!get_current_pose(current_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current pose");
            return;
        }

        // 현재 방향(yaw) 계산
        tf2::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 시계방향 45도 회전 후 직진 목표 위치 계산
        double new_yaw = yaw - M_PI / 4;  // 시계 방향으로 45도
        geometry_msgs::msg::PoseStamped goal_pose = current_pose;
        goal_pose.pose.position.x += 2.0 * cos(new_yaw);
        goal_pose.pose.position.y += 2.0 * sin(new_yaw);

        // 새로운 방향(yaw) 적용
        tf2::Quaternion new_orientation;
        new_orientation.setRPY(0.0, 0.0, new_yaw);
        goal_pose.pose.orientation.x = new_orientation.x();
        goal_pose.pose.orientation.y = new_orientation.y();
        goal_pose.pose.orientation.z = new_orientation.z();
        goal_pose.pose.orientation.w = new_orientation.w();

        RCLCPP_INFO(this->get_logger(), "Navigating to goal: x=%.2f, y=%.2f",
                    goal_pose.pose.position.x, goal_pose.pose.position.y);

        // NavigateToPose 요청 생성
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        // 요청 전송
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](auto result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Navigation failed!");
            }
        };

        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    bool get_current_pose(geometry_msgs::msg::PoseStamped &pose) {
        try {
            if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero, 5s)) {
                // RCLCPP_WARN(this->get_logger(), "Timeout waiting for transform between map and base_link");
                return false;
            }
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            pose.header = transform.header;
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.orientation = transform.transform.rotation;
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return false;
        }
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateStraight>();
    node->start_navigation();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
