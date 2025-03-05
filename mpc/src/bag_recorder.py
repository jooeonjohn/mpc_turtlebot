import rclpy
from rclpy.node import Node
import subprocess
from tf2_ros import TransformListener, Buffer
import numpy as np
import math

class ConditionalBagRecorder(Node):
    def __init__(self):
        super().__init__('conditional_bag_recorder')

        # TF2 buffer and listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bag_path = '/home/jooeon/bag2/coop/new_dwb_lab_path5'
        self.start_point = [0.6464466094067263,0.14644660940672627]
        # self.start_point = [0.5,0.0]
        
        # Internal state
        self.start_flag = False
        self.is_recording = False
        self.bag_process = None

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

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

            distance = np.sqrt((self.start_point[0]-x)**2+(self.start_point[1]-y)**2)
            if self.start_flag == False and self.is_recording == False:
                if distance < 0.1:
                    self.start_recording()
            elif self.start_flag == False and self.is_recording == True:
                if distance > 0.1:
                    self.start_flag = True
            elif self.is_recording:
                if distance < 0.1:
                    self.stop_recording()
                    


        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

    def start_recording(self):
        self.get_logger().info("Condition met. Starting bag recording...")
        self.is_recording = True
        
        # Start recording as a subprocess
        self.bag_process = subprocess.Popen(
            ['ros2', 'bag', 'record', '/cmd_vel', '/tf', '/tf_static', '/csv_path', '-o', self.bag_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

    def stop_recording(self):
        self.get_logger().info("Condition no longer met. Stopping bag recording...")
        if self.bag_process:
            self.bag_process.terminate()
            self.bag_process.wait()
            self.bag_process = None
        self.is_recording = False

    def destroy(self):
        if self.is_recording:
            self.stop_recording()
        super().destroy_node()

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

    recorder = ConditionalBagRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Shutting down node.')
    finally:
        recorder.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
