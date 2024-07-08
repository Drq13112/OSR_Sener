import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan


class ReadingLaser(Node):

    def __init__(self):
        super().__init__('reading_laser')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE



        self.subscription= self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile,
        )


    def listener_callback(self,msg):
        self.get_logger().info('I heard : Range[0] "%f" Ranges[50]: "%f"' %(msg.ranges[0] ,msg.ranges[50]))


def main(args=None):
    rclpy.init()
    reading_laser = ReadingLaser()                  
    reading_laser.get_logger().info("Hello friend!")
    rclpy.spin(reading_laser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
