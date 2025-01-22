#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class FrameFollower : public rclcpp::Node
{
public:
    FrameFollower()
        : Node("frame_follower"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer to periodically compute and send velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FrameFollower::followFrame, this));
    }

private:
    void followFrame()
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        try
        {
            // Lookup the transform from base_link to target_frame
            transformStamped = tf_buffer_->lookupTransform("base_link", "dock_frame", tf2::TimePointZero);

            // Compute linear and angular velocity commands
            double dx = transformStamped.transform.translation.x;
            double dy = transformStamped.transform.translation.y;

            auto cmd_vel = geometry_msgs::msg::Twist();
            cmd_vel.linear.x = 0.5 * dx; // Proportional control for forward movement
            cmd_vel.angular.z = 2.0 * dy; // Proportional control for rotation

            cmd_vel_pub_->publish(cmd_vel);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform dock_frame to base_link: %s", ex.what());
        }
    }

    // ROS 2 publisher and timer
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF2 Buffer and Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrameFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
