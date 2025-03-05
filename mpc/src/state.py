import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')

        # TF2 buffer and listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for the path
        self.path_publisher = self.create_publisher(Path, 'robot_path', 10)

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # State and path storage
        self.state = [0, 0, 0]  # x, y, yaw
        self.path = Path()
        self.path.header.frame_id = 'map'

    def timer_callback(self):
        try:
            # Request transform from 'map' to 'base_footprint'
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())

            # Extract position (x, y, z)
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # Extract orientation and convert to yaw
            quat = transform.transform.rotation
            yaw = self.euler_from_quaternion(quat)

            # Update state
            self.state = [x, y, yaw]

            # Log the state
            self.get_logger().info(f'X: {x}, Y: {y}, Yaw: {yaw}')

            # Create a PoseStamped for the current position
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = z
            pose_stamped.pose.orientation = quat

            # Add the pose to the path if the distance is more than 0.1
            if not self.path.poses or math.sqrt((x - self.path.poses[-1].pose.position.x)**2 + (y - self.path.poses[-1].pose.position.y)**2) > 0.1:
                self.path.poses.append(pose_stamped)

            # Publish the path
            self.path_publisher.publish(self.path)

        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

    def euler_from_quaternion(self, quat):
        # Convert quaternion to yaw (rotation angle)
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        # Calculate roll, pitch, yaw
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
