import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import numpy as np
from scipy.linalg import block_diag
from controller import mpc_controller
import csv
from geometry_msgs.msg import Twist


class MPC(Node):
    def __init__(self):
        super().__init__('mpc')
        self.path = self.load_csv('/home/jooeon/colcon_ws/src/mpc/path/path3.csv')
        self.state = [0,0,0]
        self.vel = 0
        self.omega = 0
        # TF2 버퍼와 리스너 생성
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        
        # 주기적으로 포즈를 업데이트 할 타이머 설정
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        

    def load_csv(self, filename):
        path = []
        try:
            with open(filename, mode='r') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    if row:
                        x = float(row[0])
                        y = float(row[1])
                        path.append([x,y])
        except Exception as e:
            self.get_logger().error(f"Error loading CSV file: {e}")
        
        return np.array(path)

    def timer_callback(self):
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
            
            # 출력
            # self.get_logger().info(f'X: {x}, Y: {y}, Yaw: {yaw}')

            self.mpc()
        except Exception as e:
            self.get_logger().warn(f"{e}")

    def mpc(self):
        i = self.cur_goal()
        
        Ts = 0.01
        Np = 10
        m = 3
        n = 2
        U_max = [0.22, 2.84]
        U_min = [0, -2.84]

        if i == len(self.path)-1:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_publisher.publish(cmd)

            return 0
        
        elif i+Np>=len(self.path):
            Np = len(self.path)-i-1 
            
        # Define matrices
        Q_1 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 10]])
        Q_0 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.1]])
        R_1 = np.diag([10, 10])
        R_0 = np.diag([10, 0.5])

        # Initialize Q and R
        Q = Q_0
        R = R_0

        # Create the block diagonal matrices
        for j in range(1, Np):
            Q = block_diag(Q, Q_1)
            R = block_diag(R, R_1)
        
        x0 = self.state
        v_ref = 0.1
        X_ref = np.array([[x,y,np.arctan2(y-x0[1],x-x0[0])] for x,y in self.path])
        U_ref = np.array([[v_ref,angdiff(x0[2],theta)] for _,_,theta in X_ref])
        theta0 = x0[2]
    
        # print(f"state: {self.state}, X_ref: {X_ref[i]}, U_ref: {U_ref[i]}")

        A = np.array([
            [1, 0, -v_ref * np.sin(theta0) * Ts],
            [0, 1, v_ref * np.cos(theta0) * Ts],
            [0, 0, 1]
        ])

        B = np.array([
            [np.cos(theta0) * Ts, 0],
            [np.sin(theta0) * Ts, 0],
            [0, Ts]
        ])

        w = np.array([
            v_ref * np.cos(theta0) * Ts,
            v_ref * np.sin(theta0) * Ts,
            self.omega * Ts
        ]).reshape(-1, 1)

        C = np.eye(m)
        
        init = np.array([[x0[0]-X_ref[i,0]], [x0[1]-X_ref[i,1]], [angdiff(X_ref[i,2],x0[2])]])
        # print(f"init: {init}")
        
        self.vel, self.omega = mpc_controller(i, A, B, C, w, init, Np, U_ref[i:i+Np,:], Q, R, U_min, U_max)

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

def angdiff(a, b):
    # Compute the difference
    diff = b - a
    # Ensure the difference is in the range [-pi, pi]
    diff = (diff + np.pi) % (2 * np.pi) - np.pi
    return diff


def main(args=None):
    rclpy.init(args=args)
    mpc = MPC()
    rclpy.spin(mpc)
    mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()