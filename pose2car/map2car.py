from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

from transforms3d.euler import euler2quat
import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros


class EgoPositionPublisher(Node):
    def __init__(self):
        """Initialize the ego vehicle position publisher."""
        super().__init__('car_origin_to_map')
        self.transformation_bc = tf2_ros.TransformBroadcaster(self)
        self.map_subscription = self.create_subscription(Marker, 'position_egovehicle', self.position_callback, 1)
        self.map_subscription
    
    def position_callback(self, msg):
        mc = TransformStamped()
        mc.header.frame_id = 'map'
        mc.child_frame_id = 'car_origin'
        mc.header.stamp = self.get_clock().now().to_msg()
        mc.transform.rotation.x = float(0)
        mc.transform.rotation.y = float(0)
        mc.transform.rotation.z = msg.pose.orientation.z
        mc.transform.rotation.w = msg.pose.orientation.w
        mc.transform.translation.x = msg.pose.position.x
        mc.transform.translation.y = msg.pose.position.y
        mc.transform.translation.z = float(0)
        self.transformation_bc.sendTransform(mc)

def main(args=None):
    """Start the ego position publisher node."""
    rclpy.init(args=args)
    car_origin_to_map = EgoPositionPublisher()
    try:
        rclpy.spin(car_origin_to_map)
    except KeyboardInterrupt:
        print('Shutting down car_origin_to_map node.')

    car_origin_to_map.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()