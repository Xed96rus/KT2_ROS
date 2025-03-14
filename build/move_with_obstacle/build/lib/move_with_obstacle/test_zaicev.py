import time
import rclpy
from cmath import pi
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2  

class TurtleController(Node):
    def __init__(self):
        super().__init__('cherepavel')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.move_turtle)


        self.x = 0.0
        self.y = 0.0
        self.target_x = 4.0 
        self.target_y = 4.0  
        self.distance_threshold = 0.1 
    def move_turtle(self):
        msg = Twist()


        distance = ((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2) ** 0.5
        
        if distance > self.distance_threshold:

            angle_to_target = atan2(self.target_y - self.y, self.target_x - self.x)
            angle_diff = angle_to_target - self.z
            

            angle_diff = (angle_diff + pi) % (2 * pi) - pi
            
            
            msg.linear.x = min(1.0, distance)  
            msg.angular.z = angle_diff * 2.0   
            
        else:
      
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

    def pose_callback(self, pose):
      
        self.x = pose.x
        self.y = pose.y
        self.z = pose.theta

        print("X: " + str(self.x), "Y: " + str(self.y), "Z: " + str(self.z))


def main():
    rclpy.init()
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

