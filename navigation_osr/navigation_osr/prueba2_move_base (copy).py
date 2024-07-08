#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
import numpy as np
import math

# Base class to handle exceptions
from tf2_ros import TransformException, TransformBroadcaster
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer

from tf2_ros.transform_listener import TransformListener


class LaserScanProcessor(Node):
    def __init__(self):
        super().__init__('prueba2_move_base')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.subscriber = self.create_subscription(LaserScan,'/scan',self.laser_callback,qos_profile)
        self.subscriber

        # Publicador para el marker del baricentro
        self.marker_pub = self.create_publisher(Marker, '/centroid_marker', qos_profile)

        # Publicador para los comandos de velocidad
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel',10)

        self.frame_id = "laser_frame"  # Frame adecuado del sensor láser
        self.rate = self.create_rate(100)

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

        self.current_x= None
        self.current_y= None
        self.current_yaw=None
        self.goal_yaw=None
        self.quat=None


        # Configurar un temporizador para calcular y publicar cada 0.1 segundos
        self.timer = self.create_timer(0.1, self.timer_callback)

        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.pub_movebase)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.robot_pose)

        self.publisher_goal = self.create_publisher(
      PoseStamped, 
      '/goal_pose2', 
      1)
        
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

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

        

        #self.follow_centroid()

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
        marker.header.stamp = self.get_clock().now().to_msg()
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

    def publish_mid_marker(self):
        if self.mean_mid is None:
            return

        #print(self.mean_mid)

        # Calcular el punto medio entre el centroide y el punto medio y publicarlo
        if self.centroid is not None and self.mean_mid is not None:
            #print("here")

            self.new_mid_point = ((self.centroid[0] + self.mean_mid[0]) / 2, (self.centroid[1] + self.mean_mid[1]) / 2)

            marker_mid = Marker()
            marker_mid.header.frame_id = self.frame_id
            marker_mid.header.stamp = self.get_clock().now().to_msg()
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
            marker_mid.color.g = 0.0
            marker_mid.color.b = 1.0  # Color azul (el bueno)
            self.marker_pub.publish(marker_mid)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'goal_tf'

            # Turtle only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = abs(self.new_mid_point[1])
            t.transform.translation.y = self.new_mid_point[0]
            t.transform.translation.z = 0.0

            # For the same reason, turtle can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)

            marker_seguim = Marker()
            marker_seguim.header.frame_id = self.frame_id
            marker_seguim.header.stamp = self.get_clock().now().to_msg()
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

    def timer_callback(self):
        if self.centroid:
            self.publish_marker(self.centroid)

    

    def quaternion_from_euler(self,roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def robot_pose(self):
        trans = None
     
        try:
          now = rclpy.time.Time()
          trans = self.tf_buffer.lookup_transform(
                      'map',
                      'base_link',
                      now)
          trans_goal = self.tf_buffer.lookup_transform(
                      'base_link',
                      'goal_tf',
                      now)
        except TransformException as ex:
          return

        # Publish the 2D pose
        self.current_x = trans.transform.translation.x
        self.current_y = trans.transform.translation.y

        self.goal_yaw=math.atan2((trans_goal.transform.translation.y-self.current_y),(trans_goal.transform.translation.x-self.current_x))
        self.quat = self.quaternion_from_euler (0.0, 0.0,self.goal_yaw)

    def pub_movebase(self):
        if self.new_mid_point is not None and self.quat is not None:
            msg = PoseStamped()
            msg.header.frame_id='base_link'
            msg.pose.position.x=abs(self.new_mid_point[1])
            msg.pose.position.y=self.new_mid_point[0]
            msg.pose.position.z=0.0
            msg.pose.orientation.x=self.quat[0]
            msg.pose.orientation.y=self.quat[1]
            msg.pose.orientation.z=self.quat[2]
            msg.pose.orientation.w=self.quat[3]

            self.publisher_goal.publish(msg)

            print("x: ", abs(self.new_mid_point[1]))
            print("y: ", self.new_mid_point[0])
        else:
            print("Something is failing, but im fine")
        

def main(args=None):
    rclpy.init(args=args)
    processor = LaserScanProcessor()
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
