#!/usr/bin/env python3
"""
Publish robot_description from file as a ROS2 topic message.
This bridges between robot_state_publisher (which sets a parameter)
and ros_gz_sim create (which expects a topic).
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self, sdf_file: str, namespace: str = "iris"):
        super().__init__("robot_description_publisher", namespace=namespace)
        
        # Create publisher for robot_description topic
        self.publisher = self.create_publisher(
            String, 
            "robot_description", 
            1  # Transient local QoS for durability
        )
        
        # Load and publish robot description
        if os.path.exists(sdf_file):
            with open(sdf_file, 'r') as f:
                robot_desc = f.read()
            
            msg = String()
            msg.data = robot_desc
            
            # Publish immediately and then repeatedly
            for _ in range(5):
                self.publisher.publish(msg)
                self.get_logger().info(
                    f"Published robot_description ({len(robot_desc)} bytes)"
                )
            
            # Keep node alive to maintain publisher
            self.create_timer(1.0, lambda: self.publisher.publish(msg))
        else:
            self.get_logger().error(f"SDF file not found: {sdf_file}")
            raise FileNotFoundError(f"SDF file not found: {sdf_file}")


def main():
    import sys
    
    rclpy.init()
    
    # Get SDF file path from argument or environment
    sdf_file = sys.argv[1] if len(sys.argv) > 1 else os.environ.get(
        "ROBOT_SDF_FILE",
        os.path.expanduser("~/.local/share/iris_with_gimbal/model.sdf")
    )
    
    namespace = sys.argv[2] if len(sys.argv) > 2 else "iris"
    
    node = RobotDescriptionPublisher(sdf_file, namespace)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
