
# Image Processing Node for ZED Camera

## Overview
This ROS2 node subscribes to a ZED camera topic, processes images by applying **denoising** and **edge detection**, and publishes the results. Additionally, it saves the original video feed to a file for later analysis.

## Features
- **Subscribes** to the ZED camera's rectified RGB image topic.
- **Applies Denoising** using Gaussian Blur and Non-Local Means Denoising.
- **Performs Edge Detection** using the Canny edge detector.
- **Publishes** the denoised and edge-detected images as ROS2 topics.
- **Displays** original, denoised, and edge-detected images using OpenCV.
- **Saves** the original video feed as an AVI file.

## Dependencies
first I Ensured that I have the following installed:
- **ROS2 (jazzy)**
- **OpenCV (`cv2`)**
- **`cv_bridge` for ROS2 image conversions**
- **ZED ROS2 Wrapper** (for obtaining images from the ZED camera)

## Installation
If dependencies are missing, install them using:
```bash
pip install opencv-python numpy
```

## Code Explanation

### 1. **Imports and Node Initialization**
```python
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
```
These imports provide:
- OpenCV for image processing
- ROS2 support for creating nodes and handling messages
- `cv_bridge` to convert between ROS2 and OpenCV images

### 2. **Class Definition: `ImageProcessingNode`**
```python
class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.bridge = CvBridge()
```
- This class **inherits** from `Node` to make it a ROS2 node.
- `self.bridge = CvBridge()` is used for image format conversion.

### 3. **Subscribing to the ZED Camera Image Topic**
```python
self.subscription = self.create_subscription(
    Image,
    '/zed/zed_node/rgb/image_rect_color',  # ZED Camera topic
    self.image_callback,
    10)
```
- This listens to **rectified RGB images** from the ZED camera.
- Calls `image_callback()` whenever a new image is received.
it sbscribes to the /zed/zed_node/rgb/image_rect_color topic

### 4. **Publishing Processed Images**
```python
self.denoised_publisher = self.create_publisher(Image, 'denoised_image', 10)
self.edge_publisher = self.create_publisher(Image, 'edge_detected_image', 10)
```
- Publishes:
  - **Denoised image** on `denoised_image`
  - **Edge-detected image** on `edge_detected_image`

### 5. **Saving the Original Video**
```python
self.video_writer = cv2.VideoWriter(
    'output.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (1280, 720))
```
- Saves the original camera feed to `output.avi` with **30 FPS** and **1280x720 resolution**.

### 6. **Processing the Image in `image_callback`**
```python
cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
```
- Converts the ROS2 Image message to OpenCV format.

#### **6.1 Display and Save Original Video**
```python
cv2.imshow("Original Video", cv_image)
self.video_writer.write(cv_image)
```
- Displays the original camera feed.
- Saves it to `output.avi`.

#### **6.2 Apply Denoising**
```python
denoise_img = cv2.GaussianBlur(cv_image, (3, 3), 1.5)
denoise_img = cv2.fastNlMeansDenoisingColored(denoise_img, None, 10, 10, 7, 15)
```
- **Gaussian Blur** smooths the image.
- **Non-Local Means Denoising** removes color noise while preserving details.

#### **6.3 Publish Denoised Image**
```python
denoise_msg = self.bridge.cv2_to_imgmsg(denoise_img, encoding='bgr8')
self.denoised_publisher.publish(denoise_msg)
```
- Converts the denoised image back to ROS2 format and publishes it.

#### **6.4 Apply Edge Detection**
```python
edge = cv2.Canny(denoise_img, 100, 200)
cv2.imshow("Edge Detected Video", edge)
```
- Uses **Canny Edge Detection** with thresholds **100, 200** to detect edges.

#### **6.5 Publish Edge-Detected Image**
```python
edge_msg = self.bridge.cv2_to_imgmsg(edge, encoding='mono8')
self.edge_publisher.publish(edge_msg)
```
- Converts the edge-detected image back to ROS2 format and publishes it.

#### **6.6 Prevent Freezing**
```python
cv2.waitKey(1)
```
- Keeps OpenCV windows responsive.

### 7. **Clean Up and Shutdown**
```python
def destroy_node(self):
    self.video_writer.release()
    cv2.destroyAllWindows()
    super().destroy_node()
```
- **Releases** the video writer.
- **Closes all OpenCV windows**.

### 8. **Main Function**
```python
def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down image processing node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
```
- Initializes ROS2, starts the node, and keeps it running until `Ctrl+C`.

## Running the Node

### **1. Launch the ROS2 Node**
```bash
ros2 run your_package image_processing_node
```
Replace `your_package` with the actual package name.

### **2. View Processed Images**
Use **rqt_image_view** to visualize images:
```bash
ros2 run rqt_image_view rqt_image_view
```

### **3. Verify Published Topics**
```bash
ros2 topic list
```
this imcludes:
- `/denoised_image`
- `/edge_detected_image`

#### **check for the output**
- `ros2 bag play task2_rosbag_final`
- run this in on terminal and ...
- `python3 zed_camera.py`
- now the output is shown in the following screen recording



# State Estimator and IMU Position Estimator

## Introduction
This document explains two ROS2 nodes: `position_plotter.py` and `imu_position_estimator.py`. These nodes process IMU data to estimate the position of a vehicle over time and visualize its trajectory.

## 1. State Estimator

### Overview
The `position_plotter.py` script subscribes to an estimated position topic and applies a simple Kalman filter to smooth the position data. It then visualizes the vehicle’s trajectory in 3D using Matplotlib.

### How It Works
1. **Subscribe to Position Data:** The node listens to `/estimated_position`, which provides position updates in `PointStamped` messages.
2. **Apply Kalman Filter:** The raw position data is smoothed using a simple Kalman filter.
3. **Store Filtered Data:** The filtered position values are stored for visualization.
4. **Plot the Trajectory:** Once the node shuts down, it generates a 3D plot comparing the raw and filtered trajectories.

### Code Breakdown
- **`kalman_filter(prev_estimate, measurement)`**: Implements a basic Kalman filter for smoothing position estimates.
- **`position_callback(msg)`**: Processes incoming position messages, applies filtering, and logs the filtered position.
- **`plot_trajectory()`**: Uses Matplotlib to plot the raw and filtered position data in 3D.

## 2. IMU Position Estimator

### Overview
The `imu_position_estimator.py` script estimates the vehicle's position over time by integrating IMU acceleration data. It applies a low-pass filter to reduce noise and then integrates acceleration to derive velocity and position.

### How It Works
1. **Subscribe to IMU Data:** The node listens to `/zed/zed_node/imu/data`, which provides linear acceleration values.
2. **Apply Low-Pass Filtering:** The noisy acceleration values are smoothed using a simple low-pass filter.
3. **Estimate Velocity and Position:** The script integrates acceleration to estimate velocity and integrates velocity to estimate position.
4. **Publish Position Data:** The estimated position is published to `/estimated_position` for further processing.

### Code Breakdown
- **`low_pass_filter(prev_value, new_value)`**: Applies a low-pass filter to smooth the acceleration data.
- **`imu_callback(msg)`**: Processes IMU messages, applies filtering, integrates velocity and position, and publishes the result.

#### Running the Node
first I ran the commands in three different terminals:
```sh
ros2 bag play task2_rosbag_final
```
```sh
python3 position_plotter.py
```
and the third one will be in an virual environment terminal
```sh
python3 imu_estimator.py
```

## Conclusion
These two nodes work together to estimate and visualize the vehicle's trajectory. The IMU position estimator provides real-time position updates, while the state estimator applies filtering and plots the trajectory for analysis.



# Pitch Plotter

## Introduction
The `pitch_plotter.py` script is a ROS2 node that subscribes to IMU data from a ZED camera and visualizes the pitch angle over time using a live plot. This allows real-time monitoring of the camera's tilt.

## How It Works
1. **Subscribe to IMU Data:** The node listens to the `/zed/zed_node/imu/data` topic for IMU messages.
2. **Extract Orientation Data:** It retrieves the quaternion values representing the sensor's orientation.
3. **Convert to Euler Angles:** The quaternion is converted to Euler angles, specifically extracting the pitch angle.
4. **Plot the Pitch Angle:** The pitch angle (in degrees) is plotted against time, updating dynamically in a live graph.
5. **Store and Display Values:** The last 100 values are stored for smooth visualization, and the graph updates in real-time.

## Code Breakdown
- **`imu_callback(msg)`**:
  - Extracts quaternion data from the IMU message.
  - Converts quaternion to Euler angles and extracts the pitch.
  - Converts the pitch angle from radians to degrees.
  - Stores the elapsed time and pitch angle for plotting.
  - Updates the live plot with the latest values.

- **Live Plotting:**
  - Uses `matplotlib` for real-time visualization.
  - The last 100 pitch values are retained for smooth updates.
  - The graph auto-scales based on incoming data.

## Running the Node
To execute the script, run:
```sh
ros2 bag play task2_rosbag_final
```
in one terminal and then in another terminal
```sh
python3 pitch_angle.py
```

This will launch the node, and a real-time pitch angle graph will appear, updating as new IMU data is received.and the screen casts are uploaded...

## Conclusion
This node provides a simple but effective way to visualize the ZED camera’s pitch angle over time. It can be useful for monitoring camera orientation and ensuring stable positioning in autonomous applications.



