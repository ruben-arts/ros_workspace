import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Point
import argparse

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__(node_name='coordinate_publisher')
        self.publisher_ = self.create_publisher(Point, 'coordinates', 10)

    def publish_coordinates(self, x, y):
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0  # Assuming z is not used, set to 0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x={x}, y={y}')

def main(args=None):
    # Initialize ROS with all arguments (including ROS args)
    rclpy.init(args=args)
    node = CoordinatePublisher()

    parser = argparse.ArgumentParser(description='Send coordinates over a ROS2 topic')
    parser.add_argument('x', type=float, help='X coordinate')
    parser.add_argument('y', type=float, help='Y coordinate')

    # Filter out ROS-specific arguments before parsing with argparse
    filtered_args = remove_ros_args(sys.argv)[1:]
    args = parser.parse_args(args=filtered_args if filtered_args else ['--help'])


    try:
        node.publish_coordinates(args.x, args.y)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    print("HALLO OUWE")
    main()
