import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
import math

class TrajectoryFollower(Node):

    def __init__(self):
        super().__init__('trajectory_follower')
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
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path = None
        self.current_pose = None
        self.current_goal_index = 0

    def path_callback(self, msg):
        self.get_logger().info('Received path')
        self.path = msg.poses
        self.current_goal_index = 0

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.follow_path()

    def follow_path(self):
        self.get_logger().info("Following path")
        if self.path is None or self.current_pose is None:
            return

        if self.current_goal_index >= len(self.path):
            self.get_logger().info('Reached the end of the path')
            self.stop_robot()
            return

        # Get the next goal from the path
        goal_pose = self.path[self.current_goal_index].pose

        # Compute the distance to the goal
        distance = self.euclidean_distance(self.current_pose.position, goal_pose.position)
        print("distancia a punto", goal_pose)

        # Compute the angle to the goal
        angle_to_goal = self.angle_to_goal(self.current_pose.position, goal_pose.position) # add .position
        print("distancia a angle_to_goal", angle_to_goal)

        # Convert angle to degrees for easier comparison
        angle_to_goal_degrees = math.degrees(angle_to_goal)

        # Create a Twist message
        cmd_vel = Twist()

        # Check if the robot is within the positional and angular tolerances
        if distance < 0.1 and abs(angle_to_goal_degrees) < 5:
            self.get_logger().info(f'Reached goal {self.current_goal_index}')
            self.current_goal_index += 1
            self.stop_robot()
            return

        if abs(angle_to_goal_degrees) > 10:
            # If the angular error is greater than 5 degrees, prioritize rotation
            cmd_vel.angular.z = max(min(angle_to_goal, 0.1), -0.1)
        else:
            # Simple proportional controller for linear velocity
            cmd_vel.linear.x = max(min(distance, 0.1), -0.1)
            # Simple proportional controller for angular velocity
            cmd_vel.angular.z = max(min(angle_to_goal, 0.1), -0.1)

        # Publish the velocity commands
        self.publisher_.publish(cmd_vel)

    def stop_robot(self):
        cmd_vel = Twist()
        self.publisher_.publish(cmd_vel)

    def euclidean_distance(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

    def angle_to_goal(self, current_pose, goal_pose):
        current_yaw = self.yaw_from_quaternion(current_pose.orientation) # yaw robot
        
        #goal_yaw = self.yaw_from_quaternion(goal_pose.orientation) # PROBLEMA ***
        dx = goal_pose.position.x - current_pose.position.x
        dy = goal_pose.position.y - current_pose.position.y
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - current_yaw
        print("angle_diff")

        if angle_diff >= math.pi:
            angle_diff = -2*math.pi + angle_diff
            
        if angle_diff <= -math.pi:
            angle_diff = angle_diff + 2*math.pi


        return angle_diff

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    trajectory_follower = TrajectoryFollower()
    rclpy.spin(trajectory_follower)
    trajectory_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
