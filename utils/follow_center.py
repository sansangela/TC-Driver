import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

import csv
import math
import os

class DataGathererNode(Node):
    def __init__(self):
        super().__init__('follow_center_node')
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.data = {}
        self.file_path = os.path.join(os.path.dirname(__file__), 'data.csv')
        self.id = 0  # Example ID, can be dynamic as per requirement
        with open(self.file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            # writer.writerow(['id', 's_m', 'd_m', 'x_m', 'y_m', 'd_right', 'd_left', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2'])

        self.d_left = None
        self.d_right = None

        self.prev_time_stamp = None
        self.vx_mps_previous = 0.0
        self.total_path_length = 0.0

        self.Kp = 1.0


    def lidar_callback(self, msg):
        angle_left = -135.0 * math.pi / 180 
        angle_left_max = -45.0 * math.pi / 180  # Left section from -135 to -45 degrees
        angle_right = 45.0 * math.pi / 180  # Right section from 45 to 135 degrees
        angle_right_max = 135.0 * math.pi / 180

        left_start_index = int((angle_left - msg.angle_min) / msg.angle_increment)
        left_end_index = int((angle_left_max - msg.angle_min) / msg.angle_increment)

        right_start_index = int((angle_right - msg.angle_min) / msg.angle_increment)
        right_end_index = int((angle_right_max - msg.angle_min) / msg.angle_increment)

        # Extract left and right ranges and filter out invalid measurements
        left_ranges = [r for r in msg.ranges[left_start_index:left_end_index] if r > msg.range_min and r < msg.range_max]
        right_ranges = [r for r in msg.ranges[right_start_index:right_end_index] if r > msg.range_min and r < msg.range_max]

        # Calculate minimum distances
        d_left = min(left_ranges) if left_ranges else float('inf')
        d_right = min(right_ranges) if right_ranges else float('inf')

        print(f"d_left={d_left} d_right={d_right}")

        # Update the class attribute or process further as needed
        self.d_left = d_left
        self.d_right = d_right

        track_width = d_left + d_right
        center_offset = d_right - (track_width / 2.0)

        steering_angle = self.Kp * center_offset

        # Create and publish an AckermannDrive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.steering_angle = steering_angle
        self.publisher_.publish(drive_msg)
        self.get_logger().info(f'Publishing drive message with steering angle: {steering_angle}')



    
def main(args=None):
    rclpy.init(args=args)
    node = DataGathererNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()