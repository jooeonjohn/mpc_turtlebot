import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

class CSVToPathPublisher(Node):
    def __init__(self):
        super().__init__('csv_to_path_publisher')

        # Create a publisher for the Path message
        self.publisher_ = self.create_publisher(Path, '/csv_path', 10)

        # Load CSV data
        self.path = self.load_csv('/home/jooeon/colcon_ws/src/mpc/path/lab_path5.csv')

        # Create a timer to publish the Path message periodically
        self.timer = self.create_timer(1.0, self.timer_callback)

    def load_csv(self, filename):
        path = Path()
        path.header.frame_id = 'map'  # You can set the appropriate frame_id
        
        try:
            with open(filename, mode='r') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    if row:
                        x = float(row[0])
                        y = float(row[1])
                        pose_stamped = PoseStamped()
                        pose_stamped.pose.position.x = x
                        pose_stamped.pose.position.y = y
                        pose_stamped.pose.position.z = 0.0
                        path.poses.append(pose_stamped)
            self.get_logger().info(f"Loaded {len(path.poses)} points from the CSV file.")
        except Exception as e:
            self.get_logger().error(f"Error loading CSV file: {e}")
        
        return path

    def timer_callback(self):
        # Publish the loaded path
        self.publisher_.publish(self.path)
        self.get_logger().info("Publishing path")

def main(args=None):
    rclpy.init(args=args)

    node = CSVToPathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
