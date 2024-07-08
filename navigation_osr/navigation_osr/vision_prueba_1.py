#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from geometry_msgs.msg import PoseStamped, Twist
import math

class CommandProcessor(Node):

    def __init__(self):
        super().__init__('vision_prueba1')

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.process_command)
        

        # Suscripción a los tópicos
        self.color_sub = self.create_subscription(
            Int8,
            '/color_dominance',
            self.color_callback,
            10
        )

        self.num_sub = self.create_subscription(
            String,
            'ocr_num',
            self.num_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            'current_pose',
            self.pose_callback,
            10
        )

        # Crear publicador para el comando de velocidad
        self.publisher_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        # Inicializar variables
        self.color_dominance = None
        self.num_value_ocr = None
        self.num_giro_actual = 0
        self.num_goal = 0
        pose_init = PoseStamped()
        self.orientation_goal = pose_init.pose.orientation
        self.previous_error = None
        self.msg_orientation = None
        self.color_dominance = None
        self.contador = 0
        self.bandera = True

        self.bandera_color = True

        self.bandera_giro = True
        

    def color_callback(self, msg):
        try:
            if msg.data != '' and self.bandera_color == True:
                self.color_dominance = msg.data
                self.bandera_color = False
                self.get_logger().info(f'Color {self.color_dominance}')

        except ValueError:
            self.get_logger().error('Received invalid color in color_callback')
            self.color_dominance = None

    def pose_callback(self, msg):
        self.msg_orientation = msg.pose.orientation
        if self.contador == 0:
            self.orientation_goal = self.msg_orientation
            self.contador = 1


    def num_callback(self, msg):
        try:
            if msg.data != '' and self.num_goal == 0:
                self.num_goal = 1
                self.num_value_ocr = int(msg.data)

        except ValueError:
            self.get_logger().error('Received invalid number in ocr_num')
            self.num_value_ocr = None

    def num_callback(self, msg):
        try:
            if msg.data != '' and self.num_goal == 0:
                self.num_goal = 1
                self.num_value_ocr = int(msg.data)
                self.get_logger().info(f'Value {self.num_value_ocr} times consecutively')

        except ValueError:
            self.get_logger().error('Received invalid number in ocr_num')
            self.num_value_ocr = None

    def yaw_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw  # in radians


    def process_command(self):
        cmd = Twist()
        try:
            # Si  tiene color y ocr
            if self.color_dominance is not None and self.num_value_ocr != 0:
                if self.color_dominance == 1 and self.num_goal == 1:  # Rojo
                    cmd.angular.z = -0.5  # Girar a la derecha
                    if self.bandera_giro == True:
                        self.get_logger().info(f'Giro a la derecha {self.num_value_ocr} veces')
                        self.bandera_giro = False
                elif self.color_dominance == 2 and self.num_goal == 1:  # Azul
                    cmd.angular.z = 0.5  # Girar a la izquierda
                    if self.bandera_giro == True:
                        self.get_logger().info(f'Giro a la izquierda {self.num_value_ocr} veces')
                        self.bandera_giro = False

                yaw_goal = self.yaw_from_quaternion(self.orientation_goal.x, self.orientation_goal.y, self.orientation_goal.z, self.orientation_goal.w)
                yaw_actual = self.yaw_from_quaternion(self.msg_orientation.x, self.msg_orientation.y, self.msg_orientation.z, self.msg_orientation.w)
                current_error = abs(yaw_goal - yaw_actual)

                if self.previous_error is None:
                    self.previous_error = current_error

                #self.get_logger().info(f'yaw_goal {yaw_goal}   yaw_actual {yaw_actual} current error {current_error}')

                # Lógica para detectar si ha hecho una vuelta
                if current_error> 0.1:
                    self.bandera = False

                if current_error < 0.1 and self.bandera == False:
                        self.num_giro_actual += 1
                        self.bandera = True

                self.previous_error = current_error

                # Si ya ha alcanzado las vueltas, reinicio de todo.
                if self.num_giro_actual >= self.num_value_ocr:
                    cmd.angular.z = 0.0
                    self.num_goal = 0
                    self.get_logger().info(f'Giro completado {self.num_value_ocr} veces')
                    self.num_value_ocr = 0
                    #self.num_giro_actual = 0 

            else:
                cmd.angular.z = 0.0

            #self.get_logger().info(f'giro actual{self.num_giro_actual}')
            self.publisher_cmd.publish(cmd)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    command_processor = CommandProcessor()
    rclpy.spin(command_processor)
    command_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
