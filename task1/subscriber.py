import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RiddleSubscriber(Node):
    def __init__(self):
        super().__init__('riddle_subscriber')
        self.current_topic = '/start_here'
        self.subscription = self.create_subscription(
            String,
            self.current_topic,
            self.listener_callback,
            10
        )
        self.topics_discovered = []

    def listener_callback(self, msg):
        self.get_logger().info(f"Received on {self.current_topic}: {msg.data}")
        
        if self.current_topic not in self.topics_discovered:
            self.topics_discovered.append(self.current_topic)

        next_topic = msg.data.strip()
        if next_topic.startswith('/'):
            self.switch_topic(next_topic)

    def switch_topic(self, new_topic):
        self.destroy_subscription(self.subscription)
        self.current_topic = new_topic
        self.subscription = self.create_subscription(
            String,
            self.current_topic,
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Switched to topic: {new_topic}")

def main(args=None):
    rclpy.init(args=args)
    node = RiddleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Discovered topics: {node.topics_discovered}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
