import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import csv

class FollowPathClient(Node):
    def __init__(self):
        super().__init__('csv_follow_path_client')

        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self.goal_msg = FollowPath.Goal()
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
        
        self.goal_msg.path = path
        

    def send_goal(self):
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for FollowPath action server...')

        self._action_client.send_goal_async(self.goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by FollowPath action server.')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        print("DONE")
        # self.send_goal()

def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClient()

    node.load_csv('/home/jooeon/colcon_ws/src/mpc/path/lab_path3.csv')
    # Send the path to the FollowPath action server
    node.send_goal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
