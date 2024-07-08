#!/usr/bin/env python

# CCU v 3 Jul 24

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
import math

class LaserScanProcessor(Node):
    def __init__(self):
        super().__init__('laser_scan_processor')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            qos_profile)
        
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_2', 10)
        
        self.previous_error_angular = 0.0
        self.previous_error_linear = 0.0
        self.previous_time = self.get_clock().now()
        
    def callback(self, data):
        # Definir los ángulos objetivos
        #angles = [-1.87, -1.57, -1.27, 0, 1.57]
        angles = [2.84, -3.14, -2.84, -1.57, 0]
        
        target_ranges = []

        for idx, target_angle in enumerate(angles):
            angle_min = data.angle_min
            angle_increment = data.angle_increment
            
            target_index = int((target_angle - angle_min) / angle_increment)
            #delta_index = int(math.radians(10) / angle_increment)
            delta_index = int(math.radians(15) / angle_increment)

            # Calcular los índices para el rango de 20 grados (+10 y -10) alrededor del ángulo objetivo
            start_index = max(0, target_index - delta_index)
            end_index = min(len(data.ranges) - 1, target_index + delta_index)

            # Tomar los puntos en el rango y calcular la media
            valid_ranges = [data.ranges[i] for i in range(start_index, end_index + 1) if not math.isnan(data.ranges[i])]
            if not valid_ranges:
                continue
            
            mean_distance = sum(valid_ranges) / len(valid_ranges)
            target_ranges.append(mean_distance)

            # Crear y configurar el marcador
            marker = Marker()
            #marker.header.frame_id = data.header.frame_id
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "laser_scan_mean"
            marker.id = idx  # ID único para cada marcador
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = mean_distance * math.cos(target_angle)
            marker.pose.position.y = mean_distance * math.sin(target_angle)
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            self.marker_publisher.publish(marker)

        # Controlar el movimiento del robot para mantenerlo centrado en el pasillo
        if len(target_ranges) == 5:
            distancia_frontal = target_ranges[3]
            distancia_derecha_1 = target_ranges[0]
            distancia_derecha_centro = target_ranges[1]
            distancia_derecha_3 = target_ranges[2]
            distancia_izquierda = target_ranges[4]
            

            error_lateral = distancia_derecha_3 - distancia_derecha_1
            error_frontal = distancia_frontal - 0.75

            self.get_logger().info(f"errores: {error_frontal}, {error_lateral}")
            current_time = self.get_clock().now()
            delta_time = (current_time - self.previous_time).nanoseconds / 1e9  # Convert to seconds
            
            # COMPENSACIN LATERAL

            # Controlador PD para la velocidad angular
            k_p_angular = 1.5  # Ganancia proporcional para la velocidad angular
            k_d_angular = 0.1  # Ganancia derivativa para la velocidad angular

            angular_speed = k_p_angular * error_lateral + k_d_angular * ((error_lateral - self.previous_error_angular) / delta_time)

            # COMPENSACION LINEAL 

            # Controlador PD para la velocidad lineal
            k_p_linear = 0.25  # Ganancia proporcional para la velocidad lineal
            k_d_linear = 0.05  # Ganancia derivativa para la velocidad lineal

            
            linear_speed = k_p_linear * error_frontal + k_d_linear * ((error_frontal - self.previous_error_linear) / delta_time)

            
            # OK Lineal 0.3 Angular =0.2
            max_linear_speed = 0.3
            if linear_speed > max_linear_speed:
                linear_speed = max_linear_speed
            elif linear_speed < -max_linear_speed:
                linear_speed = -max_linear_speed

            max_angular_speed = 0.2
            if angular_speed > max_angular_speed:
                angular_speed = max_angular_speed
            elif angular_speed < -max_angular_speed:
                angular_speed = -max_angular_speed


            error_centro_trayectoria =  distancia_derecha_centro - distancia_izquierda
            self.get_logger().info(f"{error_centro_trayectoria}")

            twist = Twist()
            temp_lin = linear_speed - error_centro_trayectoria * 0.4
            temp_ang = -angular_speed - error_centro_trayectoria * 0.8
            twist.linear.x = temp_lin * 0.8
            twist.angular.z = temp_ang * 0.8
            self.cmd_vel_publisher.publish(twist)

            # Actualizar errores y tiempo previo
            self.previous_error_angular = error_lateral
            self.previous_error_linear = error_frontal
            self.previous_time = current_time
        
    def start(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    processor = LaserScanProcessor()
    processor.start()
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

