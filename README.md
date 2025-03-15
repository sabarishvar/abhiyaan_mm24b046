
# introduction:
in this readme file i will be giving overview of all the methods I tried for the questions which i have not submited or the questions for which i didnt get preper answer.

## task1/subpart2:
 - first i created the turtle using the command `ros2 run turtlesim turtlesim_node`.
 - Then I used this below python code to make it go and hit the walls and then return it in random directions:
 ```#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleGame(Node):
    def __init__(self):
        super().__init__('turtle_game')

        # Publisher to move the turtle
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Subscriber to get turtle position
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        # Create a Twist message
        self.twist = Twist()

        # Start moving forward
        self.twist.linear.x = 3.0  # Move forward
        self.twist.angular.z = 1.57  # No rotation
        self.cmd_vel_pub.publish(self.twist)

    def pose_callback(self, msg):
        """Callback function to check if the turtle hits the bottom wall."""
        self.get_logger().info(f"Turtle Position - x: {msg.x}, y: {msg.y}")

        if msg.y <= 0.5:  # If turtle reaches the bottom
            self.get_logger().info("Turtle hit the bottom wall! Stopping the game.")
            
            # Stop the turtle
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            # Shutdown the node
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = TurtleGame()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```
   - by this code the first turtle will move randomly and when it hits the bottom wall the game just stops.
  - now for the second turtle I spawned it using the command `ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 1.0, theta: 0.0, name: "turtle2"}"
`
 - now to move this turtle i am going to use the below code 
```
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import termios, tty, sys, select

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle2_controller')
        
        # Publisher for turtle2 velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Subscriber to track turtle2's position
        self.pose_sub = self.create_subscription(Pose, '/turtle2/pose', self.pose_callback, 10)

        # Turtle movement speed
        self.speed = 2.0  # Adjust as needed
        self.position_x = 5.5  # Default starting x position (center)

    def pose_callback(self, msg):
        """Updates turtle2's position."""
        self.position_x = msg.x  # Update X position

    def get_key(self):
        """Detects keyboard input for movement."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def move_turtle(self):
        """Moves the turtle left and right using keyboard arrows."""
        self.get_logger().info("Use Left and Right arrow keys to move turtle2. Press 'q' to exit.")

        while rclpy.ok():
            key = self.get_key()
            twist = Twist()

            if key == '\x1b[D':  # Left Arrow Key
                twist.linear.x = -self.speed
            elif key == '\x1b[C':  # Right Arrow Key
                twist.linear.x = self.speed
            elif key == 'q':  # Quit
                self.get_logger().info("Exiting...")
                break

            self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    controller = TurtleController()
    
    try:
        controller.move_turtle()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
 ** so this is the approach i used for the turtle ping pong game **

### task5:
 - for the task 5 I just downloaded the qt creator and the qt 5 designer but for some reason it is showing error and it isnt opening.  
