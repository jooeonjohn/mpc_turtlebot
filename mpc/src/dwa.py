"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
import rclpy
from rclpy.node import Node
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
from visualization_msgs.msg import Marker, MarkerArray

class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 1.0  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.obstacle_cost_gain = 1.0

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.3  # [m] for collision check


class DWA(Node):
    def __init__(self):
        super().__init__('dwa')
        
        self.u = np.array([0.2,0.0])
        self.config = Config()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.point_subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.path_publisher = self.create_publisher(
            Path,
            '/dwa_path',
            10
        )

        self.ob_publisher = self.create_publisher(
            MarkerArray,
            '/obstacles',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def point_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        self.goal = np.array([x,y])

    def scan_callback(self, msg):
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
            # Extract angle and range data from the LaserScan message
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            ranges = msg.ranges

            # Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
            scan_coordinates = []
            for i, range_value in enumerate(ranges):
                if math.isfinite(range_value):  # Ignore invalid range values (e.g., infinity)
                    angle = yaw + angle_min + i * angle_increment
                    scan_x = x + range_value * math.cos(angle)
                    scan_y = y + range_value * math.sin(angle)
                    scan_coordinates.append((scan_x, scan_y))

            self.ob = scan_coordinates
            
        except Exception as e:
            self.get_logger().warn(f"scan callback error: {e}")
        
    def timer_callback(self):
        try:
            self.dwa()
            if hasattr(self, 'ob') and self.ob:
                marker_array = MarkerArray()

                for i, coord in enumerate(self.ob):
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "obstacles"
                    marker.id = i
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = coord[0]
                    marker.pose.position.y = coord[1]
                    marker.pose.position.z = 0.0
                    marker.scale.x = 0.1  # Sphere size
                    marker.scale.y = 0.1
                    marker.scale.z = 0.1
                    marker.color.a = 1.0  # Alpha
                    marker.color.r = 1.0  # Red
                    marker.color.g = 0.0  # Green
                    marker.color.b = 0.0  # Blue

                    marker_array.markers.append(marker)

                self.ob_publisher.publish(marker_array)
        except Exception as e:
            self.get_logger().warn(f"{e}")

    def dwa(self):
        config = self.config

        dist_to_goal = math.hypot(self.state[0] - self.goal[0], self.state[1] - self.goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            return

        x = self.state + self.u
        
        dw = self.calc_dynamic_window(x, config)

        self.u, trajectory = self.calc_control_and_trajectory(x, dw, config)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for x in trajectory:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = x[0]
            pose_stamped.pose.position.y = x[1]
            pose_stamped.pose.position.z = 0.0
            path.poses.append(pose_stamped)
        
        self.path_publisher.publish(path)
            

    def calc_dynamic_window(self, x, config):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [-config.max_yaw_rate, config.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [x[4] - config.max_delta_yaw_rate * config.dt,
            x[4] + config.max_delta_yaw_rate * config.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1])]

        return dw
    
    def calc_control_and_trajectory(self, x, dw, config):
        """
        calculation final input with dynamic window
        """
        goal = self.goal
        ob = self.ob

        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])
        v = 0.1

        # evaluate all trajectory with sampled input in dynamic window

        for y in np.arange(dw[0], dw[1], config.yaw_rate_resolution):

            trajectory = self.predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
            ob_cost = config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob, config)

            final_cost = to_goal_cost  + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                
        return best_u, best_trajectory
    
    def predict_trajectory(self, x_init, v, y, config):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= config.predict_time:
            x = self.motion(x, [v, y], config.dt)
            trajectory = np.vstack((trajectory, x))
            time += config.dt

        return trajectory
    
    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost
    
    def calc_obstacle_cost(self, trajectory, ob, config):
        """
        calc obstacle cost inf: collision
        """
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

      
        if np.array(r <= config.robot_radius).any():
            return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK
    
    def motion(self, x, u, dt):
        """
        motion model
        """

        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x

            
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
   

if __name__ == '__main__':
    rclpy.init()
    dwa = DWA()
    rclpy.spin(dwa)
    dwa.destroy_node()
    rclpy.shutdown()
