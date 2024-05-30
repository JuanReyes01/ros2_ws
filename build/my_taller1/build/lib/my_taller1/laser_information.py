import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32  # Ajusta esto según el tipo de mensaje que estés usando
from geometry_msgs.msg import Twist
import math
class LaserDataSubscriber(Node):
    def __init__(self):
        super().__init__('laser_data_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,      # El tipo del mensaje
            '/hokuyo_laser_data',   # El nombre del tópico
            self.listener_callback, # La función que se llama cuando se recibe un mensaje
            10                      # La profundidad de la cola de mensajes
        )
        self.orientation_sub = self.create_subscription(
            Float32, '/turtlebot_orientation', self.orientation_callback, 10
        )
        self.subscription = self.create_subscription(Twist,'/turtlebot_position',self.listener_callback_position,10)
        self.laser_data_pub = self.create_publisher(Float32MultiArray, '/processed_laser_data', 10)
        self.position_pub = self.create_publisher(Twist, '/processed_position', 10)
        
        # elf.position_pub = self.create_publisher(Point32, '/robot_position', 10)
        #self.regions_pub = self.create_publisher(LaserScan, '/laser_regions', 10)  # Ajustar tipo de mensaje si es necesario
        
        self.subscription  # prevent unused variable warning
        self.laser_data_list=[ ]
        self.position_laser=[ ]
        self.x_laser_transform=[ ]
        self.y_laser_transform=[ ]
        self.regions= { } #
        self.x,self.y,self.z=0.0,0.0,0.0
        self.orientation = 0
    
    def orientation_callback(self, msg):
        self.orientation = msg.data - math.pi
    
    def listener_callback(self, msg):
            # Convertir los datos de Float32MultiArray a una lista de Python
            self.laser_data_list = list(msg.data)
            #print(len(self.laser_data_list))
            #print("Datos del láser recibidos y convertidos a lista:", self.laser_data_list)
            self.position_laser,self.x_laser_transform,self.y_laser_transform=self.descompress_data(self.laser_data_list,self.x,self.y,self.orientation) 
            print( "x:",self.x,"y:",self.y,"orientation:",self.orientation)
            
            
            
           

    def descompress_data(self,lista_sensores,posx,posy,orientation):
        # matrix [i,j] donde i representa la distancia x, y j la distancia y  del sensor al robot, donde la pòsición es relativa  y no tiene en cuenta la orientación del robot
        matrix=[ ] 
        x=[ ] # lista de las distancias transformadas en el eje x con respecto a la posicion del robot  y su orientacion
        y=[ ] #lista de las distancias transformadas en el eje y con respecto a la posicion del robot y s orientacion
        for i in range(0,len(lista_sensores),2):
            fila=[ ]
            fila.append(lista_sensores[i])
            fila.append(lista_sensores[i+1])
            lx=lista_sensores[i]+posx
            ly=lista_sensores[i+1]+posy
            xt,xy=self.transform_coordinates(lx,ly,orientation,posx,posy)
            
            x.append(xt)
            y.append(xy)
            matrix.append(fila)
        return matrix,x,y

    def transform_coordinates(self,lx,ly,orientation,posx,posy):
        cth = math.cos(orientation)
        sth = math.sin(orientation)
        x_transformated = (cth * lx - sth * ly) +posx
        y_transformated = (sth * lx + cth * ly) +posy
        
        return x_transformated,y_transformated
    
    
    
    
    def listener_callback_position(self, msg):
        # Display a message on the console every time a message is received on the
        # addison topic
        self.x,self.y,self.z=msg.linear.x,msg.linear.y,msg.linear.z #Descompresiòn de coordenadas




        
    def publish_laser_data(self):
        msg = Float32MultiArray()
        msg.data = self.laser_data_list
        #print(msg.data)
        self.laser_data_pub.publish(msg)

    def publish_position(self):
        msg = Twist()
        msg.linear.x = self.x
        msg.linear.y = self.y
        msg.linear.z = self.z
        self.position_pub.publish(msg)


    def get_laser_data(self):
        return self.laser_data_list
    
    def get_position(self):
        return self.x,self.y,self.z
def main(args=None):
    rclpy.init(args=args)
    node = LaserDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()