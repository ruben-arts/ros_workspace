#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>

class TurtleNavigator : public rclcpp::Node
{
public:
    TurtleNavigator()
        : Node("turtle_navigator"), x_goal_(0.0), y_goal_(0.0), kp_(1.0), ki_(0.0), kd_(0.0), prev_error_(0.0), integral_(0.0)
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "coordinates", 10, std::bind(&TurtleNavigator::goal_callback, this, std::placeholders::_1));
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleNavigator::pose_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TurtleNavigator::control_loop, this));
    }

private:
    void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        x_goal_ = msg->x;
        y_goal_ = msg->y;
        RCLCPP_INFO(this->get_logger(), "Received goal: x=%f, y=%f", x_goal_, y_goal_);
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x_current_ = msg->x;
        y_current_ = msg->y;
        theta_current_ = msg->theta;
    }

    void control_loop()
    {
        double error_x = x_goal_ - x_current_;
        double error_y = y_goal_ - y_current_;
        double distance_error = std::sqrt(error_x * error_x + error_y * error_y);

        double angle_to_goal = std::atan2(error_y, error_x);
        double angle_error = angle_to_goal - theta_current_;

        // PID control
        double control_signal = kp_ * distance_error + ki_ * integral_ + kd_ * (distance_error - prev_error_);
        integral_ += distance_error;
        prev_error_ = distance_error;

        // Publish velocity commands
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = control_signal;
        msg.angular.z = 4.0 * angle_error; // simple P controller for angle

        publisher_->publish(msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_goal_, y_goal_;
    double x_current_, y_current_, theta_current_;
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleNavigator>());
    rclcpp::shutdown();
    return 0;
}
