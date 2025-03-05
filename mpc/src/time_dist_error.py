import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import math
import csv

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')

        self.ref_path = self.load_csv('/home/jooeon/colcon_ws/src/mpc/path/sim_test_path1.csv')

        self.errors = []
        self.time_plot = []
        self.time = 0

        # TF2 buffer and listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        
    def destroy_node(self):
        
        # Plot the data when the node is shut down
        if self.time_plot and self.errors:
            plt.figure(figsize=(10, 6))
            plt.plot(self.time_plot, self.errors, label='Distance Error')
            plt.plot(self.time_plot, [0]*len(self.time_plot), label = 'Reference Path')
            plt.xlabel('Time')
            plt.ylabel('Perpendicular Distance Error')
            plt.ylim(-0.5,0.5)
            plt.legend()
            plt.grid(True)
            plt.show()
        else:
            print("No data collected to plot.")
            
        super().destroy_node()  # Call the base class method to clean up the node

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
            # self.get_logger().info(f'X: {x}, Y: {y}, Yaw: {yaw}')
            
            self.time += 0.1 
            min_distance = float('inf')
            
            for i in range(len(self.ref_path) - 1):
                line_start = self.ref_path[i]
                line_end = self.ref_path[i + 1]
                distance, proj_distance = point_to_line_projection([x,y], line_start, line_end)
                
                if abs(distance) < abs(min_distance):
                    min_distance = distance
                  
            self.errors.append(min_distance)
            self.time_plot.append(self.time)

            print(f"plotting...({self.time},{min_distance})")


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
    
# Function to compute perpendicular distance and projection distance
def point_to_line_projection(point, line_start, line_end):
    # Vector from line_start to line_end
    line_vec = line_end - line_start
    # Vector from line_start to point
    point_vec = point - line_start
    # Line segment length squared
    line_len_sq = np.dot(line_vec, line_vec)
    
    if line_len_sq == 0:
        # Line start and end are the same point
        return 0, 0
    
    # Projection factor t
    t = np.dot(point_vec, line_vec) / line_len_sq
    if t < 0:
        # Closest to line_start
        closest_point = line_start
        proj_distance = 0
    elif t > 1:
        # Closest to line_end
        closest_point = line_end
        proj_distance = np.linalg.norm(line_end - line_start)
    else:
        # Closest point on the line segment
        closest_point = line_start + t * line_vec
        proj_distance = np.linalg.norm(closest_point - line_start)
    
    # Distance from point to closest point
    distance = np.linalg.norm(point - closest_point)

    if np.cross(point_vec, line_vec) < 0:
        distance *= -1

    return distance, proj_distance

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# # Define the reference path and actual path
# reference_path = np.array([[0, 0], [1, 1], [2, 2], [3, 3], [4, 4]])  # Example waypoints
# actual_path = np.array([[0, 0], [1.1, 0.9], [2.2, 2.1], [3.1, 2.9], [4, 4.2]])



# # Compute cumulative reference distances
# reference_distances = [0]
# for i in range(1, len(reference_path)):
#     dist = np.linalg.norm(reference_path[i] - reference_path[i - 1])
#     reference_distances.append(reference_distances[-1] + dist)
# reference_distances = np.array(reference_distances)

# # Compute errors and corresponding reference distances
# errors = []
# projected_distances = []
# for point in actual_path:
#     min_distance = float('inf')
#     proj_distance_on_ref = 0
#     for i in range(len(reference_path) - 1):
#         line_start = reference_path[i]
#         line_end = reference_path[i + 1]
#         distance, proj_distance = point_to_line_projection(point, line_start, line_end)
#         cumulative_proj_distance = reference_distances[i] + proj_distance
#         if distance < min_distance:
#             min_distance = distance
#             proj_distance_on_ref = cumulative_proj_distance
#     errors.append(min_distance)
#     projected_distances.append(proj_distance_on_ref)

# # Plotting
# plt.figure(figsize=(10, 6))
# plt.plot(projected_distances, errors, label='Distance Error')
# plt.xlabel('Cumulative Reference Path Distance')
# plt.ylabel('Perpendicular Distance Error')
# plt.title('Perpendicular Distance Error vs. Reference Path Distance')
# plt.legend()
# plt.grid(True)
# plt.show()
