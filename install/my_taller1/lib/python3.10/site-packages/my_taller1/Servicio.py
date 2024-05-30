from interface_t1.srv import MiServicio                        
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray, Float32
from abc import ABC, abstractmethod
from rclpy.qos import qos_profile_system_default
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import atexit
import sys
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import random
import math

from collections import deque
import threading

def find_nearest_empty_point(mapa, target):
    # Crear una cola para la búsqueda en amplitud
    queue = deque([target])
    # Crear un conjunto para almacenar los puntos visitados
    visited = set([target])

    while queue:
        point = queue.popleft()
        if mapa[point[0], point[1]] == 0:
            return point
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            next_point = (point[0] + dx, point[1] + dy)
            if (0 <= next_point[0] < mapa.shape[0] and 0 <= next_point[1] < mapa.shape[1] and
                next_point not in visited):
                queue.append(next_point)
                visited.add(next_point)

    # Si no se encuentra ningún punto vacío, devolver None
    return None

def custom_round(value,dec,last):
    value *= 10**(dec+1)  # Cambiar a la escala de centésimas
    fraction, whole = np.modf(value)
    if 4 <= fraction < 6:
        if last != 0:
            value = last
        else:
            value = np.round(value / (10**(dec+1)),dec)
    else:
        value = np.round(value / (10**(dec+1)),dec)
    return value  # Cambiar de nuevo a la escala original


import numpy as np


def is_near_obstacle(grid, node, distance):
    i, j = node
    rows, cols = grid.shape
    for di in range(-distance, distance + 1):
        for dj in range(-distance, distance + 1):
            ni, nj = i + di, j + dj
            if 0 <= ni < rows and 0 <= nj < cols and grid[ni][nj] == 1:
                return True
    return False


def get_neighbors(grid, node):
    i, j = node
    rows, cols = grid.shape
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    neighbors = [(i + di, j + dj) for di, dj in directions if 0 <= i + di < rows and 0 <= j + dj < cols]
    return neighbors


def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

import heapq

def a_star(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))  # La cola de prioridad almacena tuplas de (f, node)
    closed_set = set()
    parents = {start: None}
    g = {start: 0}
    direction_changes = {start: 0}
    last_direction = {start: None}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = parents[current]
            path.reverse()
            return path, g[goal]

        closed_set.add(current)

        for neighbor in get_neighbors(grid, current):
            if neighbor in closed_set:
                continue
            tentative_g_score = g[current] + 1

            if is_near_obstacle(grid, neighbor, 1):
                tentative_g_score += 100

            direction = (neighbor[0] - current[0], neighbor[1] - current[1])
            if last_direction[current] != direction:
                direction_changes[neighbor] = direction_changes[current] + 1
            else:
                direction_changes[neighbor] = direction_changes[current]

            last_direction[neighbor] = direction

            if neighbor not in g or tentative_g_score < g[neighbor]:
                parents[neighbor] = current
                g[neighbor] = tentative_g_score
                f = g[neighbor] + heuristic(neighbor, goal) + direction_changes[neighbor] * 10
                heapq.heappush(open_set, (f, neighbor))

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.iter=0
        self.callback_group = ReentrantCallbackGroup()
        print("Servicio")
        self.orientation=0
        self.orientation_sub = self.create_subscription(
            Float32, '/turtlebot_orientation', self.orientation_callback, 10
        )
        self.position_sub = self.create_subscription(
            Twist, '/turtlebot_position', self.position_callback, 10
        )
        self.laser_data_sub = self.create_subscription(
            Float32MultiArray, '/hokuyo_laser_data', self.laser_data_callback, 10
        )
        self.limitex=[]
        self.limitey=[]
        self.vel_pub = self.create_publisher(Twist,"/turtlebot_cmdVel",10)
        self.srv = self.create_service(MiServicio, 'miservicio', self.MiServicio_callback, callback_group=self.callback_group)
        self.posx_deseado,self.posy_deseado=0,0
        self.posx,self.posy=0,0
        self.position_laser, self.x_laser_robot_position, self.y_laser_robot_position = None, None, None


    def orientation_callback(self, msg):
        self.orientation = -(msg.data - 2*math.pi)
        #print(np.rad2deg(self.orientation))

    def position_callback(self, msg):
        self.posx = msg.linear.x
        self.posy = msg.linear.y
        #print("x:", self.posx, "y", self.posy)

    def laser_data_callback(self, msg):
        self.laser_data_list = list(msg.data)
        self.position_laser, self.x_laser_robot_position, self.y_laser_robot_position = self.descompress_data()

    def descompress_data(self):
        matrix = []
        x = []
        y = []
        for i in range(0, len(self.laser_data_list), 2):
            x.append(round(self.laser_data_list[i]*math.cos(self.orientation) - self.laser_data_list[i+1]*math.sin(self.orientation) + self.posx,1))
            y.append(round(self.laser_data_list[i]*math.sin(self.orientation) + self.laser_data_list[i+1]*math.cos(self.orientation) + self.posy,1))
            fila = [x[i//2],y[i//2]]
            #x.append(-self.laser_data_list[i])
            #y.append(self.laser_data_list[i + 1])
            matrix.append(fila)
        return matrix, x, y


    def MiServicio_callback(self, request, response):

        ruta=request.ruta
        print("Servicio")
        print(ruta)
        self.valores = np.arange(-8.0,8.1, 0.1)

        # Generar la matriz simétrica
        self.size = len(self.valores)
        self.mapa = np.zeros((self.size, self.size))


        if len(request.ruta)>0:
            response.confirmacion = True
        else:
            response.confirmacion=False  
                                                          # CHANGE
        self.get_logger().info('Incoming request\na: %r' % (request.ruta))  # CHANGE

        self.control_robot(str(ruta))

        return response
    def mapping_and_path_finding(self):
        valores_rounded = np.round(self.valores, 4)
        while (self.ratio_separation() > 0.6):
            obst_x = self.x_laser_robot_position
            obst_y = self.y_laser_robot_position
            for x, y in zip(obst_x, obst_y):
                x_index = np.where(valores_rounded == x)[0][0]
                y_index = np.where(valores_rounded == y)[0][0]
                if x_index.size > 0 and y_index.size > 0:
                    self.mapa[x_index - 2:x_index + 3, y_index - 2:y_index + 3] = 1
            x, y = self.posx, self.posy
            self.lastx = custom_round(x, 1, self.lastx)
            self.lasty = custom_round(y, 1, self.lastx)
            posx_index = np.where(valores_rounded == self.lastx)[0][0]
            posy_index = np.where(valores_rounded == self.lasty)[0][0]
            posx_deseado_index = np.where(valores_rounded == self.posx_deseado)[0][0]
            posy_deseado_index = np.where(valores_rounded == self.posy_deseado)[0][0]
            start = (posx_index, posy_index)
            goal = (posx_deseado_index, posy_deseado_index)
            grilla = self.mapa
            grilla[start[0], start[1]] = 0
            if grilla[goal[0], goal[1]] == 1:
                goal = find_nearest_empty_point(grilla, goal)
            self.path, c = a_star(grilla, start, goal)
            path_image = np.zeros_like(grilla)
            path_image[tuple(zip(*self.path))] = 0.5
           
            
            
           

 
    def control_robot(self, group):
        self.posx_deseado,self.posy_deseado=group.split(',')
        self.posx_deseado=np.round(float(self.posx_deseado), 1)
        self.posy_deseado=np.round(float(self.posy_deseado), 1)
        print("posx_deseado: %f posy_deseado: %f" % (self.posx_deseado, self.posy_deseado))
        self.publisher_ = self.create_publisher(Twist, '/turtlebot_cmdVel', 1)
        self.lastx=0
        self.lasty=0
        mapping_thread = threading.Thread(target=self.mapping_and_path_finding)
        mapping_thread.start()
        time.sleep(5)
        while (self.ratio_separation() > 0.6):
            try:
                print("x_deseado: %f y_deseado: %f x_actual: %f y_actual: %f " % (self.posx_deseado,self.posy_deseado,self.posx,self.posy))
                #Hacer codigo de moviemiento de roboot , recordar suscribirse a los topicos de '/turtlebot_position', /turtlebot_orientation' y '/hokuyo_laser_data''
                path=self.path
                # Calcular la dirección del siguiente punto en la ruta
                start = path[0]
                next_point = path[1]  # El primer punto es la posición actual, así que tomamos el segundo
                dx = next_point[0] - start[0]
                dy = next_point[1] - start[1]
                if dx > 0:
                    desired_angle = 0
                elif dx < 0:
                    desired_angle = 180
                elif dy > 0:
                    desired_angle = 90
                else:
                    desired_angle = -90
                difx=abs(path[min(len(path)-1,3)][0]-path[0][0])
                dify=abs(path[min(len(path)-1,3)][1]-path[0][1])
                orientation = round(math.degrees(self.orientation))
                print(f"or {orientation} | exp {desired_angle}")
                angle_difference = desired_angle - orientation
                # Crear y publicar el mensaje Twist
                msg = Twist()
                msg.angular.z=0.0
                msg.linear.x=0.0
                if angle_difference != 0:
                    msg.angular.z = max(0.1,min(0.8, 0.008*abs(angle_difference)))  # Tomar el valor absoluto de angle_difference
                    if angle_difference < 0:
                        msg.angular.z = -msg.angular.z
                else:
                    if dx != 0:
                        msg.linear.x = 0.05*difx
                    else:
                        msg.linear.x = 0.05*dify
                self.publisher_.publish(msg)
            except:
                None
        msg = Twist()
        msg.angular.z=0.0
        msg.linear.x=0.0
        self.publisher_.publish(msg)
           
    def ratio_separation(self):
        r=((self.posx-self.posx_deseado)**2+(self.posy-self.posy_deseado)**2)**(0.5)
        return r
    
    def plot_data(self):
        fig, ax = plt.subplots()
        ax.grid()
        ln, = ax.plot([], [], 'ro')
        ax.set_xlim(-8, 8)
        ax.set_ylim(-8, 8)

        def init():
            return ln,

        def update(frame):
            ln.set_data(self.x_laser_robot_position, self.y_laser_robot_position)
            return ln,

        ani = FuncAnimation(fig, update, frames=None, init_func=init, blit=True)
        plt.show()



def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    minimal_service = MinimalService()
    plot_thread = threading.Thread(target=minimal_service.plot_data)
    plot_thread.start()
    executor.add_node(minimal_service)
    executor.spin()
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
