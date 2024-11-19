#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <builtin_interfaces/msg/time.hpp>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp> 

class TransformPublisher : public rclcpp::Node
{
public:
    TransformPublisher()
        : Node("station_tf_publisher")
    {
        // Initialize the tf2 broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Load transforms from YAML file
        std::string config_path = ament_index_cpp::get_package_share_directory("station_tf_publisher") + "/config/station_pos.yaml";
        loadTransformsFromYaml(config_path);
    }

private:
    void loadTransformsFromYaml(const std::string &yaml_file)
    {
        try
        {
            // Load the YAML file
            YAML::Node config = YAML::LoadFile(yaml_file);

            // Loop over each transform in the YAML file
            for (const auto &transform_data : config["transforms"])
            {
                geometry_msgs::msg::TransformStamped transform;

                // Set the header
                transform.header.stamp = this->get_clock()->now();
                transform.header.frame_id = transform_data["frame_id"].as<std::string>();
                transform.child_frame_id = transform_data["child_frame_id"].as<std::string>();

                // Set translation
                transform.transform.translation.x = transform_data["translation"]["x"].as<double>();
                transform.transform.translation.y = transform_data["translation"]["y"].as<double>();
                transform.transform.translation.z = transform_data["translation"]["z"].as<double>();

                // Set rotation
                transform.transform.rotation.x = transform_data["rotation"]["x"].as<double>();
                transform.transform.rotation.y = transform_data["rotation"]["y"].as<double>();
                transform.transform.rotation.z = transform_data["rotation"]["z"].as<double>();
                transform.transform.rotation.w = transform_data["rotation"]["w"].as<double>();

                // Broadcast the transform
                tf_broadcaster_->sendTransform(transform);
            }
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading YAML file: %s", e.what());
        }
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPublisher>());
    rclcpp::shutdown();
    return 0;
}
