import atexit
import sys
from abc import ABC, abstractmethod

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

import io



class Teleop(Node):

    def __init__(self):
        atexit.register(self._emergency_stop)
        Node.__init__(self, "keyboard_teleop")

        self.declare_parameter("twist_stamped_enabled", False)
        self.declare_parameter("robot_base_frame", "base_link")


        vel_linear=float(input("velocidad_lineal_max: "))
        vel_angular=float(input("velocidad_angular_max: "))

        self.declare_parameter("linear_max", vel_linear)
        self.declare_parameter("angular_max", vel_angular)


        self.declare_parameter("publish_rate", 10.0)  #Tiempo de publicaciòn del nodo

        self.LINEAR_MAX = self.get_parameter("linear_max").value
        self.ANGULAR_MAX = self.get_parameter("angular_max").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value #Frame de velocidad del robot base


        if self.get_parameter("twist_stamped_enabled").value: #Se crea el publisher al topico /turtlebot_cmdVel
            self.publisher_ = self.create_publisher(
                TwistStamped, '/turtlebot_cmdVel', qos_profile_system_default
            )
            self._make_twist = self._make_twist_stamped
        else:
            self.publisher_ = self.create_publisher(
                Twist, '/turtlebot_cmdVel', qos_profile_system_default
            )
            self._make_twist = self._make_twist_unstamped


        # Tiempo de publicaciòn de los nodos en Ros2    
        rate = 1 / self.get_parameter("publish_rate").value
        self.create_timer(rate, self._publish)
        self.linear = 0.0
        self.angular = 0.0

        self.ruta=None
        self.file=None

        self.text=""

   

    def set_ruta(self, ruta):
        self.ruta = ruta

        

    def write_twist(self, linear=None, angular=None,ruta=None):
        if linear is not None:
            if abs(linear) <= self.LINEAR_MAX:
                self.linear = linear
            else:
                self.get_logger().error(
                    f"Trying to set a linear speed {linear} outside of allowed range of [{-self.LINEAR_MAX}, {self.LINEAR_MAX}]"
                )
        if angular is not None:
            if abs(angular) <= self.ANGULAR_MAX:
                self.angular = angular
            else:
                self.get_logger().error(
                    f"Trying to set a angular speed {angular} outside of allowed range of [{-self.ANGULAR_MAX}, {self.ANGULAR_MAX}]"
                )

        if ruta is not None:
            self.ruta = ruta
        self._update_screen()

    def _make_twist_unstamped(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _make_twist_stamped(self, linear, angular):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self._robot_base_frame
        twist_stamped.twist = self._make_twist_unstamped(linear, angular)
        return twist_stamped

    def _publish(self):
        twist = self._make_twist(self.linear, self.angular)
        self.publisher_.publish(twist)

        print("ruta teleop: ",self.ruta)

        if self.ruta is not None:
            print(self.text)
            self.text+=f"{self.linear:.2f} {self.angular:.2f}\n"




    def get_text(self):
        return self.text
    def _update_screen(self):
        sys.stdout.write(f"Linear: {self.linear:.2f}, Angular: {self.angular:.2f}\r")
        
       


    def _emergency_stop(self):
        self.publisher_.publish(self._make_twist(0.0, 0.0))
