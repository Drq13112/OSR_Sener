import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
import math

class FollowPath(Node):

    def __init__(self):
        super().__init__('path_follower')

        timer_period = 0.1  # seconds
        self.create_timer(timer_period, self.path_follow)

        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10)
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10)
        
        self.publisher_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path = None
        self.current_pose = None
        self.current_goal_index = 0
        self.orientation_goal=[]
        self.tolerance_yaw=0.2
        self.tolerance_distance=0.15
        self.max_linear=0.03
        self.max_angular=0.1
        self.kp_linear=0.2
        self.kp_angular=0.8

    def path_callback(self, msg):
        self.get_logger().info('Received path')
        self.path = msg.poses
        self.current_goal_index = 1

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        

    def path_follow(self):
        if self.path is not None and self.current_goal_index<len(self.path):
            self.orientation_goal=self.angle_cal(self.path[self.current_goal_index-1].pose,self.path[self.current_goal_index].pose)
            current_yaw=self.yaw_from_quaternion(self.current_pose.orientation.x,self.current_pose.orientation.y,self.current_pose.orientation.z,self.current_pose.orientation.w)
            error_yaw=self.orientation_goal-current_yaw
            error_x=self.euclidean_distance(self.path[self.current_goal_index].pose,self.current_pose)


            print("Error yaw: ",error_yaw)
            print("Punto",self.current_goal_index)
            #ganancias del controlador
            
            if error_yaw >= math.pi:
                error_yaw = -2*math.pi + error_yaw
            
            if error_yaw <= -math.pi:
                error_yaw = error_yaw + 2*math.pi

            #aqui van las senales de control
            vel_x=max(min(self.kp_linear*error_x, self.max_linear), -self.max_linear)
            vel_theta=max(min(self.kp_angular*error_yaw, self.max_angular), -self.max_angular)

            if ((abs(error_yaw)>=self.tolerance_yaw) and error_x>=self.tolerance_distance):
                #aqui va el mensaje a publicar
                cmd=Twist()
                cmd.linear.x=0.0
                cmd.angular.z=vel_theta
            elif ((abs(error_yaw)<self.tolerance_yaw) and error_x>=self.tolerance_distance):
                
                cmd=Twist()
                cmd.linear.x=vel_x
                cmd.angular.z=vel_theta
            else:
                cmd=Twist()
                self.current_goal_index+=1
                

            self.publisher_cmd.publish(cmd)
        else:
            cmd=Twist()
            self.publisher_cmd.publish(cmd)
            self.path = None

    def angle_cal(self,p1,p2):
        if (p2.position.x!=p1.position.x):
            angle_dif=math.atan2((p2.position.y-p1.position.y),(p2.position.x-p1.position.x))
            


        else:
            angle_dif=0
        return angle_dif
    
    def yaw_from_quaternion(self,x, y, z, w):
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw # in radians

    def euclidean_distance(self, pos1, pos2):
        return math.sqrt((pos1.position.x - pos2.position.x) ** 2 + (pos1.position.y - pos2.position.y) ** 2)
    

def main(args=None):
    rclpy.init(args=args)
    followpath1 = FollowPath()
    rclpy.spin(followpath1)
    followpath1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
