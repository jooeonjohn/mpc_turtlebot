import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import csv
import numpy as np
from tf2_ros import TransformListener, Buffer

class FollowPathClient(Node):
    def __init__(self):
        super().__init__('csv_follow_path_client')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self.goal_msg = FollowPath.Goal()
        self.load_csv('/home/jooeon/colcon_ws/src/mpc/path/lab_path5.csv')
        # Send the path to the FollowPath action server
        self.timer = self.create_timer(1.0, self.send_goal)
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
        
        self.path = path.poses
        

    def send_goal(self):
        try:
            # "base_link"에서 "map"으로의 변환을 요청
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # 변환된 포즈 (x, y, z, roll, pitch, yaw)
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # 회전 정보 (쿼터니언을 오일러 각도로 변환)
            quat = transform.transform.rotation
            yaw = self.euler_from_quaternion(quat)

            self.state = [x,y,yaw]
            goal_ind = self.cur_goal()

            if goal_ind + 10 > len(self.path):
                additional_ind = goal_ind + 10 - len(self.path)
                self.goal_msg.path.poses = self.path[goal_ind:] + self.path[1:additional_ind + 1]
            else:
                self.goal_msg.path.poses = self.path[goal_ind:goal_ind + 10]
            
            while not self._action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Waiting for FollowPath action server...')

            self._action_client.send_goal_async(self.goal_msg).add_done_callback(self.goal_response_callback)

        except Exception as e:
            self.get_logger().warn(f"{e}")

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

    def cur_goal(self):
        goal_rad = 0.1
        current_x, current_y = self.state[0], self.state[1]
        

        path = self.path
        # print(path)
        
        # Find the nearest point in the path
        distances = [np.sqrt((current_x - pose.pose.position.x) ** 2 + (current_y - pose.pose.position.y) ** 2) for pose in path]
        goal_index = np.argmin(distances)
        goal_point = path[goal_index]

        while distances[goal_index]<goal_rad:
            goal_index += 1
            if goal_index >= len(self.path):
                goal_index = 1
            goal_point = path[goal_index]

        # print(f"target: {goal_index}, {goal_point}")
     
        return goal_index
    
    def euler_from_quaternion(self, quat):
        # 쿼터니언을 yaw(회전각)으로 변환
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        
        # Roll, Pitch, Yaw 계산
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClient()

    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
