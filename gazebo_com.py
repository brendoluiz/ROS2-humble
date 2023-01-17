#!/usr/bin/env python3
import math
import rclpy  
import geometry_msgs
import nav_msgs
import tf_transformations

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from transforms3d import euler,quaternions

class gazeboControl(Node):
    
    def __init__(self):
        super().__init__("gazebo_com")
        self.destino_x = 8.0
        self.destino_y = 4.0

        self.pose_ = None
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "vehicle_blue/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Odometry, "vehicle_blue/odometry", self.callback_vehicle_blue, 10)
        self.control_signal_timer_ = self.create_timer(
            0.01, self.control_signal)

    def callback_vehicle_blue(self, msg):
        self.pose_ = msg

    def control_signal(self):
        if self.pose_ == None:
            return

        dist_x = self.destino_x - self.pose_.position.x
        dist_y = self.destino_y - self.pose_.position.y
        distance = math.sqrt(math.pow(dist_x, 2) + math.pow(dist_y, 2))
        angle_dist = math.atan2(dist_y, dist_x)
        orientation = angle_dist - self.pose_.pose.pose.orientation.z
        if orientation > math.pi:
            orientation -= 2*math.pi
        elif orientation < -math.pi: 
            orientation += 2*math.pi

        msg = Twist()

        if distance > 0.1:
            #posição
            msg.linear.x = 2*distance
            #orientação
            msg.angular.z = 6*orientation

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # calculos

        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    
    rclpy.init(args=args) #Inicializa a infra ROS2 para esse nó específico
    node = gazeboControl() # Cria o objeto Node para a classe py_1 (dá um nome ao nó)
    rclpy.spin(node) # Pausa o programa aqui, mantendo-o ativo e permitindo lidar com callbacks. Senão a linha shutdown é executada
    rclpy.shutdown() # destrói o nó e limpa a memoria

if __name__ == "__main__":  # Estrutura padrão Python para o main
    main()