import rclpy
import numpy as np
import pyproj
import math
import time
import scipy.spatial.transform
from rclpy.node import Node
from marvelmind_interfaces.msg import HedgePos, HedgeImuFusion
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker as mk


class pubsub(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(mk, '/position_egovehicle', 10)
        self.subscription_pos = self.create_subscription(HedgePos, 'hedge_pos', self.position_callback, 10)
        self.subscription_imu = self.create_subscription(HedgeImuFusion, 'hedge_imu_fusion', self.imu_callback, 10)
        self.currentTimeStamp = self.get_clock().now().to_msg()

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.orient = 0.0
        self.timestamp = 0
        self.imu_data = None
        
        # Static marker data
        self.carmodel = mk()
        self.carmodel.ns = "CarModel"
        self.carmodel.id = 21                  
        self.carmodel.type = mk.MESH_RESOURCE
        #self.carmodel.type = mk.ARROW
        #self.carmodel.type = mk.SPHERE
        self.carmodel.action = mk.ADD
        self.carmodel.header.frame_id = 'map'
        self.carmodel.scale.x = 0.001
        self.carmodel.scale.y = 0.001
        self.carmodel.scale.z = 0.001
        self.carmodel.color.a = 1.0
        self.carmodel.color.r = 1.0
        self.carmodel.color.g = 0.0
        self.carmodel.color.b = 0.0
        self.carmodel.mesh_resource = "file:///home/rishi/icp_ws/src/map_und_pos/pose2car/models/car_new.stl"

    def imu_callback(self, msg):
        self.imu_data = msg
            
    def orientation(self):
        #if self.imu_data is None:
        #    return 0.0
        euler = scipy.spatial.transform.Rotation.from_quat([self.imu_data.qx, self.imu_data.qy, self.imu_data.qz, self.imu_data.qw]).as_euler('zyx')  # converting IMU queternion to euler 
        o = euler[0] # taking z as main rotating angle
        return o

    def position_callback(self, msg):
        xPos = msg.x_m # postion from headge_pos in x and y
        yPos = msg.y_m

        self.d_x = xPos - self.x
        self.d_y = yPos - self.y

        travelDistance = math.sqrt(self.d_x ** 2 + self.d_y ** 2)

        # new position
        self.x = xPos
        self.y = yPos

        if self.d_x != 0 or self.d_y != 0:  # Check if the car has moved
            if travelDistance > 0.10:
                self.orient = self.orientation() - 1.5707 # there is offset of 130 degree 
                self.update_marker()
        self.update_marker()        # Update after the condition
    

    def update_marker(self):
        self.carmodel.header.stamp = self.currentTimeStamp
        self.carmodel.pose.position.x = round(self.x, 2)
        self.carmodel.pose.position.y = round(self.y, 2)
        self.carmodel.pose.position.z = 0.4
        quat = quaternion_from_euler(1.5707, 0, self.orient)
        self.carmodel.pose.orientation.x = quat[0]
        self.carmodel.pose.orientation.y = quat[1]
        self.carmodel.pose.orientation.z = quat[2] 
        self.carmodel.pose.orientation.w = quat[3]
        self.publisher_.publish(self.carmodel)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = pubsub()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

