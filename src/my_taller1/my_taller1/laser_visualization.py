import math
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist

class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('robot_visualizer')
        
        self.laser_data_sub = self.create_subscription(
            Float32MultiArray, '/hokuyo_laser_data', self.laser_data_callback, 10
        )
        
        self.orientation_sub = self.create_subscription(
            Float32, '/turtlebot_orientation', self.orientation_callback, 10
        )
        self.position_sub = self.create_subscription(
            Twist, '/turtlebot_position', self.position_callback, 10
        )
        self.orientation = 0
        self.x, self.y, self.z = 0, 0, 0
        self.x_position_robot = 0
        self.y_position_robot = 0
        
        self.position_laser = []
        self.laser_data_list = []
        self.x_laser_robot_position = []
        self.y_laser_robot_position = []
        self.scale = 65
        self.scale1 = 1
        self.offset_angle = -math.pi
        self.init_pygame()

    def init_pygame(self):
        pygame.init()
        self.width, self.height = 800, 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Robot Orientation Visualization')
        self.clock = pygame.time.Clock()

    def orientation_callback(self, msg):
        self.orientation = msg.data + math.pi

    def position_callback(self, msg):
        self.x, self.y, self.z = msg.linear.x * -self.scale1, msg.linear.y * self.scale1, msg.linear.z
        self.x_position_robot = msg.linear.x
        self.y_position_robot = msg.linear.y
        print("x:", self.x, "y", self.y)

    def laser_data_callback(self, msg):
        self.laser_data_list = list(msg.data)
        self.position_laser, self.x_laser_robot_position, self.y_laser_robot_position = self.descompress_data()

    def descompress_data(self):
        matrix = []
        x = []
        y = []
        for i in range(0, len(self.laser_data_list), 2):
            fila = [self.laser_data_list[i], self.laser_data_list[i + 1]]
            x.append(-self.laser_data_list[i] * self.scale + self.x)
            y.append(self.laser_data_list[i + 1] * self.scale + self.y)
            matrix.append(fila)
        return matrix, x, y

    def render_text(self, text, position):
        font = pygame.font.Font(None, 25)
        text_surface = font.render(text, True, (255, 255, 255))
        self.screen.blit(text_surface, position)

    def radianes_a_grados(self, radianes):
        grados = radianes * (180 / math.pi)
        return grados

    def draw_points_sensor(self):
        center_x, center_y = self.width // 2, self.height // 2
        cth = math.cos(self.orientation + self.offset_angle)
        sth = math.sin(self.orientation + self.offset_angle)

        print("laser_points_sensor_y=", self.y_laser_robot_position)
        for i in range(len(self.position_laser)):
            l_x = self.x_laser_robot_position[i]
            l_y = self.y_laser_robot_position[i]
            world_x = (cth * l_x - sth * l_y) + center_x
            world_y = (sth * l_x + cth * l_y) + center_y
            pygame.draw.circle(self.screen, (255, 100, 0), (world_x, world_y), 2)

    def run_visualization(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.screen.fill((0, 0, 0))
            center_x, center_y = self.width // 2, self.height // 2

            robot_screen_x = center_x + self.x
            robot_screen_y = center_y + self.y

            print("robot_screen_x=", robot_screen_x)
            print("robot_screen_y=", robot_screen_y)
            print("orientation=", self.orientation)
            pygame.draw.circle(self.screen, (0, 0, 255), (robot_screen_x, robot_screen_y), 10)
            
            orient_length = 20
            orient_x = robot_screen_x + orient_length * math.cos(self.orientation)
            orient_y = robot_screen_y + orient_length * math.sin(self.orientation)
            pygame.draw.line(self.screen, (255, 255, 0), (robot_screen_x, robot_screen_y), (orient_x, orient_y), 2)
            pygame.draw.circle(self.screen, (255, 255, 0), (orient_x, orient_y), 5)
            
            self.render_text(f'x: {round(self.x_position_robot,2)}', (50, 50))
            self.render_text(f'y: {round(self.y_position_robot,2)}', (200, 50))
            self.render_text(f'orientation: {round(self.radianes_a_grados(self.orientation-2*math.pi))}', (350, 50))
            self.draw_points_sensor()
            
            pygame.display.flip()
            self.clock.tick(60)

            # Handle ROS subscriptions
            rclpy.spin_once(self, timeout_sec=0.01)

        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    visualizer = RobotVisualizer()
    try:
        visualizer.run_visualization()
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
