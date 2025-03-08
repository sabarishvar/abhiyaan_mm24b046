import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from tf_transformations import euler_from_quaternion
import math
import time

class PitchPlotter(Node):
    def __init__(self):
        super().__init__('pitch_plotter')
        self.subscription = self.create_subscription(
            Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)
        
        # Store timestamps and pitch values
        self.timestamps = []
        self.pitch_values = []
        self.start_time = time.time()

        # Setup live plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], label="Pitch Angle (deg)")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Pitch (degrees)")
        self.ax.set_title("ZED Camera Pitch Angle Over Time")
        self.ax.legend()

    def imu_callback(self, msg):
        # Extract quaternion from IMU message
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w

        # Convert quaternion to Euler angles
        _, pitch, _ = euler_from_quaternion([qx, qy, qz, qw])

        # Convert pitch from radians to degrees
        pitch_degrees = math.degrees(pitch)

        # Get time since start
        elapsed_time = time.time() - self.start_time

        # Store values
        self.timestamps.append(elapsed_time)
        self.pitch_values.append(pitch_degrees)

        # Keep only the last 100 values for smooth updating
        if len(self.timestamps) > 100:
            self.timestamps.pop(0)
            self.pitch_values.pop(0)

        # Update plot
        self.line.set_xdata(self.timestamps)
        self.line.set_ydata(self.pitch_values)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

        self.get_logger().info(f'Pitch Angle: {pitch_degrees:.2f}°')

def main():
    rclpy.init()
    node = PitchPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    plt.ioff()  # Turn off interactive mode
    plt.show()

if __name__ == '__main__':
    main()
