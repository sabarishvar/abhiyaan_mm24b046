import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.bridge = CvBridge()

        # Subscribe to the ZED camera topic
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',  # Updated topic name
            self.image_callback,
            10)
        
        # Publishers for processed images
        self.denoised_publisher = self.create_publisher(Image, 'denoised_image', 10)
        self.edge_publisher = self.create_publisher(Image, 'edge_detected_image', 10)

        # Video Writer for saving original video
        self.fps = 30
        self.width = 1280  # Update based on your ZED camera resolution
        self.height = 720
        self.video_writer = cv2.VideoWriter(
            'output.avi', cv2.VideoWriter_fourcc(*'XVID'), self.fps, (self.width, self.height)
        )

    def image_callback(self, msg):
        # Convert ROS2 Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ---- Display Original Video ----
        cv2.imshow("Original Video", cv_image)
        self.video_writer.write(cv_image)  # Save the original video

        # ---- Denoise Image ----
        denoise_img = cv2.GaussianBlur(cv_image, (3, 3), 1.5)
        denoise_img = cv2.fastNlMeansDenoisingColored(denoise_img, None, 10, 10, 7, 15)
        cv2.imshow("Denoised Video", denoise_img)

        # Publish Denoised Image
        denoise_msg = self.bridge.cv2_to_imgmsg(denoise_img, encoding='bgr8')
        self.denoised_publisher.publish(denoise_msg)

        # ---- Edge Detection ----
        edge = cv2.Canny(denoise_img, 100, 200)
        cv2.imshow("Edge Detected Video", edge)

        # Publish Edge Detected Image
        edge_msg = self.bridge.cv2_to_imgmsg(edge, encoding='mono8')
        self.edge_publisher.publish(edge_msg)

        cv2.waitKey(1)  # Necessary for OpenCV to update windows

    def destroy_node(self):
        self.video_writer.release()  # Save and close video file
        cv2.destroyAllWindows()
        super().destroy_node()

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

if __name__ == '__main__':
    main()
