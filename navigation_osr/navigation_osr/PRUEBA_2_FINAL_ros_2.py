#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math


class LaserScanProcessor:
    def __init__(self):
        self.node = rclpy.create_node('prueba_2')
        
        timer_period = 1.0  # seconds
        #self.timer = self.create_timer(timer_period, self.pub_movebase)

        """self.publisher_2d_pose = self.create_publisher(
      PoseStamped, 
      '/current_pose', 
      1)"""

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.subscriber = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile)

        # Publicador para el marker del baricentro
        self.marker_pub = self.node.create_publisher(Marker, '/centroid_marker', qos_profile)

        # Publicador para los comandos de velocidad
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel',10)

        self.frame_id = "laser_frame"  # Frame adecuado del sensor láser
        self.rate = self.node.create_rate(100)

        # Constantes del controlador P y D
        self.kp_dist = 2.0
        self.kp_angle = 2.0
        self.kd_angle = 0.1  # Ganancia derivativa para el error de orientación

        # Inicializar centroid y puntos laterales
        self.centroid = None
        self.mean_left = None
        self.mean_right = None
        self.mean_mid = None
        self.new_mid_point = None
        self.last_error_angle = 0.0  # Variable para almacenar el último error de orientación

        # Configurar un temporizador para calcular y publicar cada 0.1 segundos
        self.timer = self.node.create_timer(0.001, self.timer_callback)

    def laser_callback(self, data):
        # evaluacion de los distancias
        points = []
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        
        # Ajustar los ángulos para que correspondan al láser orientado 90 grados
        # COMPENSACION DEL GRADO DE LA ROTACION DEL LASER
        """if self.angle_min > np.pi:
            self.angle_min -= 2 * np.pi
        if self.angle_max > np.pi:
            self.angle_max -= 2 * np.pi"""

        # Rango frontal de 180 grados
        start_index = int((0) / self.angle_increment)
        end_index = int((np.pi) / self.angle_increment)

        for i in range(start_index, end_index):
            angle = self.angle_min + i * self.angle_increment
            distance = data.ranges[i]
            if not np.isinf(distance) and not np.isnan(distance):
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                points.append((x, y))

        if points:
            self.centroid = self.calculate_centroid(points)

        # MEDICIONES FRONTALES Y LATERALES DEL ROBOT
        #angles = [-1.57, 1.57]
        angles = [0, 3.14]
        target_ranges = []

        for idx, target_angle in enumerate(angles):
            angle_min = data.angle_min
            angle_increment = data.angle_increment

            target_index = int((target_angle - angle_min) / angle_increment)
            delta_index = int(math.radians(3) / angle_increment)

            # Calcular los índices para el rango de 20 grados (+10 y -10) alrededor del ángulo objetivo
            start_index = max(0, target_index - delta_index)
            end_index = min(len(data.ranges) - 1, target_index + delta_index)

            # Tomar los puntos en el rango y calcular la media
            valid_ranges = [data.ranges[i] for i in range(start_index, end_index + 1) if not math.isnan(data.ranges[i])]
            if not valid_ranges:
                continue

            mean_distance = sum(valid_ranges) / len(valid_ranges)
            target_ranges.append(mean_distance)
            

            # Guardar las medias de los puntos laterales
            if idx == 0:
                self.mean_left = (mean_distance * math.cos(target_angle), mean_distance * math.sin(target_angle))
            elif idx == 1:
                self.mean_right = (mean_distance * math.cos(target_angle), mean_distance * math.sin(target_angle))
        

        # Calcular el punto medio entre los puntos laterales
        if self.mean_left is not None and self.mean_right is not None:
            #print("mean_distance",self.mean_left,self.mean_right)
            self.mean_mid = ((self.mean_left[0] + self.mean_right[0]) / 2, (self.mean_left[1] + self.mean_right[1]) / 2)
            self.publish_mid_marker()

        

        self.follow_centroid()

    def calculate_centroid(self, points):
        # Convertir a numpy array
        points = np.array(points)

        # Calcular el baricentro del polígono
        centroid_x = np.mean(points[:, 0])
        centroid_y = np.mean(points[:, 1])

        return (centroid_x, centroid_y)

    def publish_marker(self, centroid):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "centroid_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

    def publish_mid_marker(self):
        if self.mean_mid is None:
            return

        print(self.mean_mid)

        # Calcular el punto medio entre el centroide y el punto medio y publicarlo
        if self.centroid is not None and self.mean_mid is not None:
            print("here")

            self.new_mid_point = ((self.centroid[0] + self.mean_mid[0]) / 2, (self.centroid[1] + self.mean_mid[1]) / 2)

            marker_mid = Marker()
            marker_mid.header.frame_id = self.frame_id
            marker_mid.header.stamp = self.node.get_clock().now().to_msg()
            marker_mid.ns = "new_mid_point_marker"
            marker_mid.id = 1
            marker_mid.type = Marker.SPHERE
            marker_mid.action = Marker.ADD
            marker_mid.pose.position.x = self.new_mid_point[0]
            marker_mid.pose.position.y = self.new_mid_point[1]
            marker_mid.pose.position.z = 0.0
            marker_mid.pose.orientation.x = 0.0
            marker_mid.pose.orientation.y = 0.0
            marker_mid.pose.orientation.z = 0.0
            marker_mid.pose.orientation.w = 1.0
            marker_mid.scale.x = 0.2
            marker_mid.scale.y = 0.2
            marker_mid.scale.z = 0.2
            marker_mid.color.a = 1.0
            marker_mid.color.r = 0.0
            marker_mid.color.g = 1.0
            marker_mid.color.b = 1.0  # Color azul para el nuevo punto medio
            self.marker_pub.publish(marker_mid)

            marker_seguim = Marker()
            marker_seguim.header.frame_id = self.frame_id
            marker_seguim.header.stamp = self.node.get_clock().now().to_msg()
            marker_seguim.ns = "centroid_marker_2"
            marker_seguim.id = 2
            marker_seguim.type = Marker.SPHERE
            marker_seguim.action = Marker.ADD
            marker_seguim.pose.position.x = self.mean_mid[0]
            marker_seguim.pose.position.y = self.mean_mid[1]
            marker_seguim.pose.position.z = 0.0
            marker_seguim.pose.orientation.x = 0.0
            marker_seguim.pose.orientation.y = 0.0
            marker_seguim.pose.orientation.z = 0.0
            marker_seguim.pose.orientation.w = 1.0
            marker_seguim.scale.x = 0.1
            marker_seguim.scale.y = 0.1
            marker_seguim.scale.z = 0.1
            marker_seguim.color.a = 1.0
            marker_seguim.color.r = 1.0
            marker_seguim.color.g = 0.0
            marker_seguim.color.b = 0.0

        
            self.marker_pub.publish(marker_seguim)


    def follow_centroid(self):
        if self.centroid is None or self.new_mid_point is None:
            return

        # Calcular el error de distancia respecto al centroide
        error_distance = math.sqrt(self.new_mid_point[0]**2 + self.new_mid_point[1]**2)

        # Calcular el error de orientación respecto al nuevo punto medio calculado
        error_angle = math.atan2(self.new_mid_point[1], self.new_mid_point[0])
        print(error_angle,error_distance)

        # Calcular la derivada del error de orientación
        current_error_angle = error_angle
        derivative_error_angle = (current_error_angle - self.last_error_angle) / 0.1  # Derivada con dt = 0.1s
        self.last_error_angle = current_error_angle

        # Controlador PD para la distancia y la orientación
        linear_speed = self.kp_dist * error_distance
        angular_speed = self.kp_angle * error_angle + self.kd_angle * derivative_error_angle

        # Publicar los comandos de velocidatimer_callback
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed * 0.3
        cmd_vel.angular.z = -angular_speed * 0.3
        self.cmd_vel_pub.publish(cmd_vel)

    def timer_callback(self):
        if self.centroid:
            self.publish_marker(self.centroid)

    #def pub_movebase(self,goal_x,goal_y):



    def start(self):
        rclpy.spin(self.node)

def main(args=None):
    rclpy.init(args=args)
    processor = LaserScanProcessor()
    processor.start()
    processor.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
