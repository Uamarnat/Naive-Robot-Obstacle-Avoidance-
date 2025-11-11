import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")
        self.threshold = 0.5 # obstacle avoidance at 0.5m
        self.cmd_publish = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.speed = 0.5
        self.angle = 1.0

    def scan_callback(self, msg: LaserScan):

        front_angle = 30
        angle_increment = math.degrees(msg.angle_increment)
        points = int(front_angle/angle_increment)
        middle = len(msg.ranges) // 2

        front_range = msg.ranges[middle - points: middle+points]

        min_distance = min(front_range)

        twist = Twist()
        if min_distance > self.threshold:
            twist.linear.x = self.speed 
            # twist.angular.z = self.angle
        else:
            twist.linear.x = 0.0
            twist.angular.z = self.angle
        
        self.cmd_publish.publish(twist)

def main(args = None):
    rclpy.init(args = args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()