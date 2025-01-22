#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

class IRDockingLocator : public rclcpp::Node
{
public:
    IRDockingLocator()
        : Node("ir_docking_locator")
    {
        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscription to IR signal strength topic
        ir_signal_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ir_signal_strength", 10, std::bind(&IRDockingLocator::irSignalCallback, this, std::placeholders::_1));

        // Timer for periodic docking logic
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&IRDockingLocator::dockUsingIR, this));
    }

private:
    void irSignalCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        ir_signal_strength_ = msg->data;
    }

    void dockUsingIR()
    {
        auto cmd_vel = geometry_msgs::msg::Twist();

        switch (state_)
        {
        case SCANNING:
            // Rotate to scan for the strongest signal
            cmd_vel.angular.z = 0.5;
            if (ir_signal_strength_ > signal_threshold_)
            {
                // Strong signal found, transition to moving towards the dock
                state_ = MOVING_TO_DOCK;
                RCLCPP_INFO(this->get_logger(), "Signal detected, moving towards dock.");
            }
            break;

        case MOVING_TO_DOCK:
            // Move forward and adjust heading to maximize signal
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = signal_gain_ * (max_signal_ - ir_signal_strength_);
            if (ir_signal_strength_ > docking_threshold_)
            {
                // Close enough to dock
                state_ = DOCKED;
                RCLCPP_INFO(this->get_logger(), "Docking complete!");
            }
            break;

        case DOCKED:
            // Stop the robot
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            break;
        }

        // Publish velocity commands
        cmd_vel_pub_->publish(cmd_vel);
    }

    enum DockingState
    {
        SCANNING,
        MOVING_TO_DOCK,
        DOCKED
    };

    // ROS 2 publisher, subscriber, and timer
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ir_signal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // IR signal strength data
    float ir_signal_strength_ = 0.0;
    float max_signal_ = 1.0;  // Replace with the maximum possible signal strength

    // Docking state machine
    DockingState state_ = SCANNING;

    // Thresholds and gains
    const float signal_threshold_ = 0.5;  // Signal strength to start moving
    const float docking_threshold_ = 0.8; // Signal strength to consider docking complete
    const float signal_gain_ = 2.0;       // Gain to adjust heading based on signal
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IRDockingLocator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
