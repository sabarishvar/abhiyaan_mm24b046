import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class StateEstimator(Node):
    def __init__(self):
        super().__init__('state_estimator')

        # Subscribe to estimated position topic
        self.subscription = self.create_subscription(
            PointStamped,
            '/estimated_position',
            self.position_callback,
            10)

        # Lists to store raw and filtered position data
        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.time_data = []

        # Kalman filter variables
        self.x_filtered = []
        self.y_filtered = []
        self.z_filtered = []

        # Process noise and measurement noise (Tune for better results)
        self.process_noise = 0.05  # Assumed noise in the system
        self.measurement_noise = 0.1  # Sensor noise
        self.estimated_error = 1.0  # Initial estimated error

    def kalman_filter(self, prev_estimate, measurement):
        """Simple Kalman Filter for position estimation"""
        kalman_gain = self.estimated_error / (self.estimated_error + self.measurement_noise)
        new_estimate = prev_estimate + kalman_gain * (measurement - prev_estimate)
        self.estimated_error = (1 - kalman_gain) * self.estimated_error + self.process_noise
        return new_estimate

    def position_callback(self, msg):
        if not self.time_data:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Get current time
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.time_data.append(current_time - self.start_time)

        # Raw position data
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        # Apply Kalman filter
        filtered_x = self.kalman_filter(self.x_filtered[-1] if self.x_filtered else x, x)
        filtered_y = self.kalman_filter(self.y_filtered[-1] if self.y_filtered else y, y)
        filtered_z = self.kalman_filter(self.z_filtered[-1] if self.z_filtered else z, z)

        self.x_filtered.append(filtered_x)
        self.y_filtered.append(filtered_y)
        self.z_filtered.append(filtered_z)

        self.get_logger().info(f'Filtered Position: x={filtered_x:.2f}, y={filtered_y:.2f}, z={filtered_z:.2f}')

    def plot_trajectory(self):
        """Plot the raw and filtered vehicle trajectory in 3D"""
        plt.figure(figsize=(10, 6))
        ax = plt.axes(projection='3d')

        # Raw trajectory
        ax.plot3D(self.x_data, self.y_data, self.z_data, 'r--', label="Raw Trajectory")

        # Smoothed trajectory (Kalman Filter)
        ax.plot3D(self.x_filtered, self.y_filtered, self.z_filtered, 'b-', label="Filtered Trajectory")

        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('Vehicle 3D Trajectory (State Estimation)')
        ax.legend()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down state estimator...")
    finally:
        node.plot_trajectory()  # Plot the trajectory before exiting
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
