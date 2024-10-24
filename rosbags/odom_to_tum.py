import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import csv

class OdomToTUM(Node):
    def __init__(self):
        super().__init__('odom_to_tum_node')
        
        # Create a subscriber to the /odom (estimated) topic
        self.subscriber_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback_estimated, 10)
        
        # Set up TF2 listener for ground truth from the /tf topic
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Open two CSV files for writing the TUM format data
        self.csv_file_estimated = open('estimated_trajectory.tum', 'w', newline='')  # Ensure no extra newlines
        self.csv_writer_estimated = csv.writer(self.csv_file_estimated, delimiter=' ')
        
        self.csv_file_ground_truth = open('ground_truth_trajectory.tum', 'w', newline='')  # Ensure no extra newlines
        self.csv_writer_ground_truth = csv.writer(self.csv_file_ground_truth, delimiter=' ')
        
        # Create a timer to periodically get the ground truth from /tf
        self.timer = self.create_timer(0.1, self.lookup_ground_truth)  # 10 Hz timer for ground truth lookup
    
    def odom_callback_estimated(self, msg):
        # Extract timestamp and pose data for the estimated trajectory
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Write estimated odom data in TUM format with exactly 8 values per row
        self.csv_writer_estimated.writerow([
            "{:.9f}".format(timestamp),  # Timestamp with 9 decimal places
            "{:.9f}".format(position.x), "{:.9f}".format(position.y), "{:.9f}".format(position.z),
            "{:.9f}".format(orientation.x), "{:.9f}".format(orientation.y), "{:.9f}".format(orientation.z), "{:.9f}".format(orientation.w)
        ])
    
    def lookup_ground_truth(self):
        # Try to look up the transform from 'map' to 'base_link'
        try:
            # Request the latest available transform with extended timeout (1 second)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), rclpy.time.Duration(seconds=3.0))
            
            # Extract timestamp and pose data from the transform
            timestamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            position = transform.transform.translation
            orientation = transform.transform.rotation

            # Write ground truth data in TUM format with exactly 8 values per row
            self.csv_writer_ground_truth.writerow([
                "{:.9f}".format(timestamp),  # Timestamp with 9 decimal places
                "{:.9f}".format(position.x), "{:.9f}".format(position.y), "{:.9f}".format(position.z),
                "{:.9f}".format(orientation.x), "{:.9f}".format(orientation.y), "{:.9f}".format(orientation.z), "{:.9f}".format(orientation.w)
            ])
        
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform: {str(e)}")
    
    def destroy(self):
        # Close both CSV files when finished
        self.csv_file_estimated.close()
        self.csv_file_ground_truth.close()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTUM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
