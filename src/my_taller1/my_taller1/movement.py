
# ROS Client Library for Python
import rclpy
 
# Handles the creation of nodes
from rclpy.node import Node
 


def main(args=None):
     
  # Initialize the rclpy library
  rclpy.init(args=args)
 
  # Create a subscriber
  movement_subscriber = Movement()
  

  # Spin the node so the callback function is called.c
  # Pull messages from any topics this node is subscribed to.
  rclpy.spin(movement_subscriber)
  #plt.show()
 
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  pygame.quit()
  movement_subscriber.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()