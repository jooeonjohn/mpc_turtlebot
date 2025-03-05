import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import numpy as np
from scipy.linalg import block_diag
from controller import mpc_controller
import csv
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PID_controller:
    def __init__(self, kp, ki, kd, i_min=None, i_max=None):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_min = i_min
        self.i_max = i_max

        # Internal variables for computation
        self.previous_error = 0
        self.integral = 0

    def compute(self, error, dt):
        
        # Proportional term
        p = self.kp * error

        # Integral term
        self.integral += error * dt
        i = self.ki * self.integral
        
        # Apply integral boundaries if defined
        if self.i_min is not None:
            i = max(i, self.i_min)
        if self.i_max is not None:
            i = min(i, self.i_max)

        # Derivative term
        d = self.kd * (error - self.previous_error) / dt

        # Compute total output
        output = p + i + d

        # Update state
        self.previous_error = error

        return output
    
class PID(Node):
    def __init__(self):
        super().__init__('mpc')
        self.path = np.array([])
        self.state = [0,0,0]
        self.vel = 0
        self.omega = 0
        # self.pid = PID_controller(7.8, 18.7, 0.8,-0.5,0.5) # Ziegler-Nichols classic
        # self.pid = PID_controller(4.29, 10.33, 1.18,-0.5,0.5) # Ziegler-Nichols some overshoot
        self.pid = PID_controller(2.6, 6.26, 0.71,-0.5,0.5) # Ziegler-Nichols no overshoot
        # self.pid = PID_controller(1.0,0.1,0.1,-0.5,0.5)
        # self.pid = PID_controller(13.0,0.0,0.0,-0.5,0.5) # Critical gain
        # TF2 버퍼와 리스너 생성
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(Path, '/csv_path', self.path_callback, 10)
        self.subscription
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        
        # 주기적으로 포즈를 업데이트 할 타이머 설정
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
    
    def path_callback(self, msg):
        try:
            self.path = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])
        except Exception as e:
            self.get_logger().error(f"Error processing path message: {e}")

    def timer_callback(self):
        try:
            # "base_link"에서 "map"으로의 변환을 요청
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            
            # 변환된 포즈 (x, y, z, roll, pitch, yaw)
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # 회전 정보 (쿼터니언을 오일러 각도로 변환)
            quat = transform.transform.rotation
            yaw = self.euler_from_quaternion(quat)

            self.state = [x,y,yaw]

            self.pid_()
        except Exception as e:
            self.get_logger().warn(f"{e}")

    def pid_(self):
               
        i = self.cur_goal()

        # # stop at end point
        # if i == len(self.path)-1:
        #     cmd = Twist()
        #     cmd.linear.x = 0.0
        #     cmd.angular.z = 0.0
        #     self.cmd_publisher.publish(cmd)

        #     return 0
        
        U_max = [0.22, 2.84]
        U_min = [0, -2.84]

        dy = self.path[i,1] - self.state[1]
        dx = self.path[i,0] - self.state[0]

        tar_angle = np.arctan2(dy,dx)
        ang_error = self.normalize_angle(tar_angle - self.state[2])
        self.omega = min(U_max[1],max(U_min[1],self.pid.compute(ang_error,0.1)))
        
        self.vel = 0.15

        print(f"vel: {self.vel}, omega: {self.omega}")

        cmd = Twist()
        cmd.linear.x = self.vel
        cmd.angular.z = self.omega
        self.cmd_publisher.publish(cmd)     

    def cur_goal(self):
        goal_rad = 0.1
        current_x, current_y = self.state[0], self.state[1]
        
        # Find the nearest point in the path
        distances = [np.sqrt((current_x - x) ** 2 + (current_y - y) ** 2) for x, y in self.path]
        goal_index = np.argmin(distances)
        goal_point = self.path[goal_index]

        while distances[goal_index]<goal_rad:
            goal_index += 1
            if goal_index >= len(self.path):
                goal_index = 1
            goal_point = self.path[goal_index]

        print(f"target: {goal_index}, {goal_point}")
     
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
    
    def normalize_angle(self, angle):

        return (angle + np.pi) % (2 * np.pi) - np.pi    # (-pi,pi)

def angdiff(a, b):
    # Compute the difference
    diff = b - a
    # Ensure the difference is in the range [-pi, pi]
    diff = (diff + np.pi) % (2 * np.pi) - np.pi
    return diff


def main(args=None):
    rclpy.init(args=args)
    mpc = PID()
    rclpy.spin(mpc)
    mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()