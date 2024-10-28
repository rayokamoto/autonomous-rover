import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import csv
import os
from datetime import datetime
import math
import tf2_ros
import tf2_geometry_msgs

class PathFollowerAndOdomToTUM(Node):
    def __init__(self):
        super().__init__('path_follower_and_odom_to_tum_node')

        # Create a directory for saving TUM files
        self.directory = os.path.join('src', 'autonomous-rover', 'benchmarks', 'mapping_benchmark')
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_directory = os.path.join(self.directory, timestamp)
        os.makedirs(self.output_directory, exist_ok=True)

        # Create a publisher for commanding the rover
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a subscriber to the /odom topic
        self.subscriber_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback_estimated, 10
        )

        # Initialize the TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Open CSV files for writing TUM format data
        estimated_file_path = os.path.join(self.output_directory, 'estimated_trajectory.tum')
        ground_truth_file_path = os.path.join(self.output_directory, 'ground_truth_trajectory.tum')

        self.csv_file_estimated = open(estimated_file_path, 'w', newline='')
        self.csv_writer_estimated = csv.writer(self.csv_file_estimated, delimiter=' ')

        self.csv_file_ground_truth = open(ground_truth_file_path, 'w', newline='')
        self.csv_writer_ground_truth = csv.writer(self.csv_file_ground_truth, delimiter=' ')

        # Waypoints (x, y)
        self.waypoints = [
            (1.550826, 1.550826),   # Start
            (-2.0, 7.0),            # Waypoint 1 (north-west)
            (-5.5, 5.0),            # Waypoint 2 (zig-zag south-west)
            (-8.0, 3.0),            # Waypoint 3 (zig-zag further south-west)
            (-10.0, 1.0),           # Waypoint 4 (zig-zag north-west)
            (-12.0, -1.5),          # Waypoint 5 (far south)
            (-10.0, -4.0),          # Waypoint 6 (north-east)
            (-6.0, -6.0),           # Waypoint 7 (zig-zag south-east)
            (-4.0, -8.0),           # Waypoint 8 (further south-east)
            (-2.0, -6.5),           # Waypoint 9 (straight back north)
            (1.550826, 1.550826)    # Return to Start
        ]
        
        self.current_waypoint_index = 0
        self.current_position = (0.0, 0.0)
        self.current_orientation = 0.0  # yaw in radians
        self.last_timestamp = None  # To store the last timestamp from odometry

        # Timer for periodic ground truth logging
        self.create_timer(0.1, self.log_ground_truth_periodically)

    def odom_callback_estimated(self, msg):
        # Extract timestamp and pose data for the estimated trajectory
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Write estimated odom data in TUM format
        self.csv_writer_estimated.writerow([
            "{:.9f}".format(timestamp),
            "{:.9f}".format(position.x), "{:.9f}".format(position.y), "{:.9f}".format(position.z),
            "{:.9f}".format(orientation.x), "{:.9f}".format(orientation.y), "{:.9f}".format(orientation.z), "{:.9f}".format(orientation.w)
        ])
        self.csv_file_estimated.flush()

        # Update the current position and orientation (convert quaternion to yaw)
        self.current_position = (position.x, position.y)
        self.current_orientation = self.quaternion_to_yaw(orientation)
        
        # Store the last timestamp for later use in ground truth logging
        self.last_timestamp = timestamp

        # Command the rover to follow the path
        self.follow_path()

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw (heading) angle."""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def follow_path(self):
        if self.current_waypoint_index < len(self.waypoints):
            target_waypoint = self.waypoints[self.current_waypoint_index]
            distance = self.calculate_distance(self.current_position, target_waypoint)

            if distance < 0.1:  # If close enough to the waypoint
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}: {target_waypoint}")
                self.current_waypoint_index += 1
            else:
                # Calculate the velocity command to move towards the waypoint
                self.move_towards_waypoint(target_waypoint)

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    def move_towards_waypoint(self, target):
        twist = Twist()
        # Calculate angle to the target waypoint
        angle_to_target = math.atan2(target[1] - self.current_position[1], target[0] - self.current_position[0])
        
        # Calculate the angle difference
        angle_diff = angle_to_target - self.current_orientation
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # Proportional control for angular velocity
        k_angular = 2.0  # Gain for angular velocity
        k_linear = 0.2    # Gain for linear velocity

        # Set linear velocity based on distance to target
        twist.linear.x = min(k_linear * self.calculate_distance(self.current_position, target), 0.2)  # Limit max speed
        twist.angular.z = k_angular * angle_diff  # Turn towards the target

        self.publisher.publish(twist)

    def log_ground_truth_periodically(self):
        """Log the ground truth data at regular intervals."""
        if self.last_timestamp is not None:
            # Determine the nearest waypoints based on the current position
            if self.current_waypoint_index > 0:
                prev_waypoint = self.waypoints[self.current_waypoint_index - 1]
                if self.current_waypoint_index < len(self.waypoints):
                    next_waypoint = self.waypoints[self.current_waypoint_index]
                    # Calculate the dynamic position using interpolation
                    self.log_dynamic_ground_truth(prev_waypoint, next_waypoint)
                else:
                    # Log the last waypoint if it's the final destination
                    self.log_ground_truth(prev_waypoint, self.last_timestamp)
            else:
                # Log the first waypoint if at the start
                self.log_ground_truth(self.waypoints[0], self.last_timestamp)

    def log_dynamic_ground_truth(self, prev_waypoint, next_waypoint):
        """Log the current position dynamically calculated between waypoints."""
        # Linear interpolation based on the distance to the next waypoint
        total_distance = self.calculate_distance(prev_waypoint, next_waypoint)
        if total_distance == 0:  # Prevent division by zero
            return
        
        # Find the ratio of how far we are between the two waypoints
        distance_to_prev = self.calculate_distance(self.current_position, prev_waypoint)
        ratio = distance_to_prev / total_distance

        # Calculate the interpolated position
        interpolated_x = prev_waypoint[0] + (next_waypoint[0] - prev_waypoint[0]) * ratio
        interpolated_y = prev_waypoint[1] + (next_waypoint[1] - prev_waypoint[1]) * ratio

        # Log the interpolated position
        self.log_ground_truth((interpolated_x, interpolated_y), self.last_timestamp)

    def log_ground_truth(self, target_waypoint, timestamp):
        """Log the ground truth data for the waypoints."""
        try:
            # Fetch the transform from the map frame to the base_link frame
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            z_coordinate = transform.transform.translation.z  # Get the z-coordinate from the transformation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform: {e}")
            z_coordinate = 0.0  # Default to 0 if unable to get transform

        # Write the ground truth data in the specified format
        self.csv_writer_ground_truth.writerow([
            "{:.9f}".format(timestamp),  # Use the synchronized timestamp
            "{:.9f}".format(target_waypoint[0]), "{:.9f}".format(target_waypoint[1]),
            "{:.9f}".format(z_coordinate),  # Current Z position from transform
            "0.0", "0.0", "0.0", "0.0"  # Identity quaternion
        ])
        self.csv_file_ground_truth.flush()  # Ensure data is written to the file

    def destroy(self):
        # Close CSV files when finished
        self.csv_file_estimated.close()
        self.csv_file_ground_truth.close()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerAndOdomToTUM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
