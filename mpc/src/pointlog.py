import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
import csv

# CSV file path
csv_file_path = '/home/jooeon/colcon_ws/src/mpc/path/pathlog.csv'

class PointLoggerNode(Node):
    def __init__(self):
        super().__init__('point_logger_node')
        
        # Create a subscriber to /clicked_point topic (PointStamped)
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )

        # Create a publisher to publish the whole path (nav_msgs/Path)
        self.publisher = self.create_publisher(
            Path,
            '/published_path',
            10
        )

        # List to store the clicked points (PointStamped)
        self.clicked_points = []

        # Create a Path message that will be published
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"  # Set the frame
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

    def point_callback(self, msg):
        # Append the received PointStamped to the list
        self.clicked_points.append(msg)
        
        # Log the received point (coordinates x, y)
        self.get_logger().info(f"Received point: {msg.point.x}, {msg.point.y}")
        
        # Publish the whole path (all the PoseStamped messages in the list)
        self.publish_path()

        # Write the latest point's x and y to the CSV file
        self.write_to_csv(msg)

    def publish_path(self):
        # Clear the previous path messages
        self.path_msg.poses.clear()

        # Populate the path message with PoseStamped
        for point in self.clicked_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = point.header
            pose_stamped.pose.position.x = point.point.x
            pose_stamped.pose.position.y = point.point.y
            pose_stamped.pose.position.z = point.point.z

            # Append to the path
            self.path_msg.poses.append(pose_stamped)

        # Publish the path
        self.publisher.publish(self.path_msg)
        self.get_logger().info(f"Publishing path with {len(self.path_msg.poses)} points")

    def write_to_csv(self, point):
        # Open the CSV file in append mode and write only x, y of the latest point
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Writing only x, y
            writer.writerow([point.point.x, point.point.y])

def main(args=None):
    rclpy.init(args=args)

    # Create the PointLoggerNode
    point_logger_node = PointLoggerNode()

    # Spin the node to keep it alive
    rclpy.spin(point_logger_node)

    # Clean up
    point_logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
