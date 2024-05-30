from interface_t1.srv import MiServicio  # CHANGE

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
from rclpy.qos import qos_profile_system_default
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import numpy as np

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.callback_group = ReentrantCallbackGroup()

        self.get_logger().info("Servicio iniciado")

        self._positionSubscription = self.create_subscription(
            Twist,
            '/turtlebot_position',
            self.listener_position_data,
            10,
            callback_group=self.callback_group
        )

        self._laserSubscription = self.create_subscription(
            Float32MultiArray,
            '/hokuyo_laser_data',
            self.listener_laser_data,
            10,
            callback_group=self.callback_group
        )

        self._positionSubscription = self.create_subscription(
            Float32,
            '/turtlebot_orientation',
            self.listener_orientation_data,
            10,
            callback_group=self.callback_group
        )

        self.srv = self.create_service(
            MiServicio,
            'miservicio',
            self.MiServicio_callback,
            callback_group=self.callback_group
        )

        self.mapping = np.zeros((800,800))
        self.posx_deseado, self.posy_deseado = 0, 0
        self.posx, self.posy, self.posz = 0, 0, 0
        self.position_laser = []
        self.laser_data_list = []
        self.x_laser_robot_position = []
        self.y_laser_robot_position = []
        self.scale = 65
        self.scale1 = 1
        self.offset_angle = -math.pi
        self.orientation = 0

        self.publisher_ = self.create_publisher(Twist, '/turtlebot_cmdVel', 1)

    def MiServicio_callback(self, request, response):
        ruta = request.ruta
        self.get_logger().info("Servicio solicitado con ruta: %s" % ruta)

        if len(ruta) > 0:
            response.confirmacion = True
        else:
            response.confirmacion = False

        self.control_robot(ruta)
        return response

    def control_robot(self, group):
        self.posx_deseado, self.posy_deseado = map(float, group.split(','))
        timer_period = 0.1  # seconds

        while self.ratio_separation() > 0.5:
            
            
            v, w = 0.0, 0.0
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w

            self.publisher_.publish(msg)
            time.sleep(timer_period)

    def ratio_separation(self):
        r = ((self.posx - self.posx_deseado) ** 2 + (self.posy - self.posy_deseado) ** 2) ** 0.5
        return r
    
    def listener_orientation_data(self,msg):
        self.orientation = msg.data 
        print(self.orientation)
    
    def listener_position_data(self, msg):
        self.posx, self.posy, self.posz = msg.linear.x, msg.linear.y, msg.linear.z

    def listener_laser_data(self, msg):
        self.laser_data_list = list(msg.data)
        #print(len(self.laser_data_list))
        self.position_laser,self.x_laser_transform,self.y_laser_transform=self.descompress_data(self.laser_data_list,self.posx,self.posy,self.orientation) 
        #print( "x:",len(self.x_laser_transform),"y:",len(self.y_laser_transform)) 
    

    def plot_data(self):
        fig, ax = plt.subplots()
        ax.set_title("Taller 1")
        ax.grid()
        ln, = ax.plot([], [], 'ro')
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)

        def init():
            return ln,

        def update(frame):
            ln.set_data(self.x_laser_transform, self.y_laser_transform)
            return ln,

        ani = FuncAnimation(fig, update, frames=None, init_func=init, blit=True)
        plt.show()

        
    def descompress_data(self):
            matrix = []
            x = []
            y = []
            for i in range(0, len(self.laser_data_list), 2):
                n = math.sqrt(self.laser_data_list[i]**2+self.laser_data_list[i+1]**2)
                thetha = math.atan2(self.laser_data_list[i],self.laser_data_list[i+1])
                
                # Coordenadas originales con orientación y posición aplicadas
                original_x = n * math.cos(thetha + self.orientation) + self.posy
                original_y = n * math.sin(thetha + self.orientation) + self.posx
                
                # Rotación -π/2 (90 grados en sentido horario)
                rotated_x = round(-original_y,2)
                rotated_y = round(-original_x,2)
                
                
                x.append(rotated_x)
                y.append(rotated_y)
                
                self.mapping[int(rotated_x)*10][int(rotated_y)*10] = 1
                
                #print(self.mapping)
            
            return x, y

    def transform_coordinates(self,lx,ly,orientation,posx,posy):
        cth = math.cos(orientation)
        sth = math.sin(orientation)
        x_transformated = (cth * lx - sth * ly) +posx
        y_transformated = (sth * lx + cth * ly) +posy
        
        return x_transformated,y_transformated

 


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    plot_thread = threading.Thread(target=minimal_service.plot_data)
    plot_thread.start()
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_service)

    try:
        executor.spin()
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()