import rclpy

from geometry_msgs.msg import Twist
from pynput import keyboard as kb
# Handles the creation of nodes
from rclpy.node import Node

class keyboard_movement(Node):
  """
  Create a subscriber node
  """
  def __init__(self):
 
    # Initiate the Node class's constructor and give it a name
    super().__init__('nodo_movement')
 
    # The node subscribes to messages of type std_msgs/String, 
    # over a topic named: /addison
    # The callback function is called as soon as a message is received.
    # The maximum number of queued messages is 10.
    self.publisher_ = self.create_publisher(Twist, '/turtlebot_cmdVel', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
 
def timer_callback(self):
        msg = Twist()
      
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
 
def main(args=None):
 
  # Initialize the rclpy library
  rclpy.init(args=args)
 
  # Create a subscriber
  movement_publisher = keyboard_movement()
 
  # Spin the node so the callback function is called.
  # Pull messages from any topics this node is subscribed to.
  rclpy.spin(movement_publisher)
 
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  movement_publisher.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()