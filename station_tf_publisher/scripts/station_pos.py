#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import TransformStamped
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time


class TransformPublisher(Node):
    def __init__(self):
        super().__init__('transform_publisher')

        self.br = tf2_ros.StaticTransformBroadcaster(self)

        # Get package share directory
        package_name = 'station_tf_publisher' 
        package_share_directory = get_package_share_directory(package_name)

      
        yaml_file = f'{package_share_directory}/config/station_pos.yaml'

       
        self.load_transforms_from_yaml(yaml_file)

    def load_transforms_from_yaml(self, yaml_file):
        try:
           
            with open(yaml_file, 'r') as file:
                transforms_data = yaml.safe_load(file)

            
            for transform_data in transforms_data['transforms']:
                transform = TransformStamped()

                
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = transform_data['frame_id']
                transform.child_frame_id = transform_data['child_frame_id']

          
                transform.transform.translation.x = transform_data['translation']['x']
                transform.transform.translation.y = transform_data['translation']['y']
                transform.transform.translation.z = transform_data['translation']['z']

              
                transform.transform.rotation.x = transform_data['rotation']['x']
                transform.transform.rotation.y = transform_data['rotation']['y']
                transform.transform.rotation.z = transform_data['rotation']['z']
                transform.transform.rotation.w = transform_data['rotation']['w']

     
                self.br.sendTransform(transform)

        except FileNotFoundError as e:
            self.get_logger().error(f"YAML file not found: {e}")
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TransformPublisher()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
