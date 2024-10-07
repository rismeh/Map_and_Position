import csv
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import Imu
from marvelmind_interfaces.msg import HedgePos


bag_file_path = '/home/rishi/icp_ws/src/icp_ris_juli_4/icp_ris_juli_4_0.db3'  # Replace with the path to your Marvelmind ROS 2 bag file
output_csv_file = '/home/rishi/icp_ws/src/icp_ris_juli_4/combined_data.csv'  # Path to the output CSV file

hedge_pos_topic = '/hedge_pos'  # Replace with the actual hedge_pos topic name in your bag file
imu_topic = '/imu_topic'  # Replace with the actual IMU topic name in your bag file

hedge_pos_data = []
imu_data = []

def hedge_pos_callback(msg):
    hedge_pos_data.append({
        'timestamp': msg.header.stamp,
        'x': msg.x,
        'y': msg.y,
        'z': msg.z,
        # Add any other hedge_pos fields you want to extract
    })

def imu_callback(msg):
    imu_data.append({
        'timestamp': msg.header.stamp,
        'linear_acceleration_x': msg.linear_acceleration.x,
        'linear_acceleration_y': msg.linear_acceleration.y,
        'linear_acceleration_z': msg.linear_acceleration.z,
        'angular_velocity_x': msg.angular_velocity.x,
        'angular_velocity_y': msg.angular_velocity.y,
        'angular_velocity_z': msg.angular_velocity.z,
        'orientation_x': msg.orientation.x,
        'orientation_y': msg.orientation.y,
        'orientation_z': msg.orientation.z,
        'orientation_w': msg.orientation.w,
    })

# Initialize the ROS 2 context
rclpy.init()

# Create a ROS 2 node
# Create a ROS 2 node
node = rclpy.create_node('data_extractor_node')


qos_profile = rclpy.qos.QoSProfile(depth=10)
hedge_pos_subscriber = node.create_subscription(HedgePos, hedge_pos_topic, hedge_pos_callback, 10)

# Create IMU subscriber
imu_subscriber = node.create_subscription(Imu, imu_topic, imu_callback, qos_profile)

# Open the bag file
bag = rclpy.create_node('ros2 bag')

try:
    with bag.open(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == hedge_pos_topic:
                hedge_pos_callback(msg)
            elif topic == imu_topic:
                imu_callback(msg)
finally:
    bag.destroy_node()
    rclpy.shutdown()

# Match hedge_pos and IMU data based on timestamps
combined_data = []
for hedge_pos_entry in hedge_pos_data:
    hedge_pos_timestamp = hedge_pos_entry['timestamp']
    closest_imu_entry = min(imu_data, key=lambda x: abs(x['timestamp'] - hedge_pos_timestamp))

    combined_entry = {
        'timestamp': hedge_pos_timestamp,
        'x': hedge_pos_entry['x'],
        'y': hedge_pos_entry['y'],
        'z': hedge_pos_entry['z'],
        # Add any other fields you want to include from hedge_pos

        'linear_acceleration_x': closest_imu_entry['linear_acceleration_x'],
        'linear_acceleration_y': closest_imu_entry['linear_acceleration_y'],
        'linear_acceleration_z': closest_imu_entry['linear_acceleration_z'],
        'angular_velocity_x': closest_imu_entry['angular_velocity_x'],
        'angular_velocity_y': closest_imu_entry['angular_velocity_y'],
        'angular_velocity_z': closest_imu_entry['angular_velocity_z'],
        'orientation_x': closest_imu_entry['orientation_x'],
        'orientation_y': closest_imu_entry['orientation_y'],
        'orientation_z': closest_imu_entry['orientation_z'],
        'orientation_w': closest_imu_entry['orientation_w'],
    }
    combined_data.append(combined_entry)

# Write the combined data to a CSV file
with open(output_csv_file, 'w') as csvfile:
    fieldnames = combined_data[0].keys()
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(combined_data)

print("Combined data extracted and saved to", output_csv_file)

