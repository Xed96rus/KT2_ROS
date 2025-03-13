import time
import rclpy
from cmath import pi
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
from math import atan2, radians  

class TurtleController(Node):
    def __init__(self):
        super().__init__('cherepavel')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.msg_subscriber = self.create_subscription(String, 'test1', self.listener_callback, 10)
        self.timer = self.create_timer(0.1, self.move_turtle)

        self.coord_obs = [[2.0, 2.5, 4.0], [5.0, 7.5, 8.0]] # X, Y
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.TARGETS = self.coord_obs
        self.target_x = 5.5
        self.target_y = 8.0
        self.target_theta = radians(45)
        self.flag = 0
        self.distance_threshold = 0.1
    def make_targets(self):
        for i in range(len(self.coord_obs[0])):
                for j in range(2):
                    if self.coord_obs[j][i] > 5.5:
                        self.TARGETS[j][i] = self.coord_obs[j][i] - 1.0
                    elif self.coord_obs[j][i] < 5.5:
                        self.TARGETS[j][i] = self.coord_obs[j][i] + 1.0
    def move_targets(self):

        msg = Twist()
        distance = ((self.TARGETS[0][self.flag] - self.x) ** 2 + (self.TARGETS[1][self.flag] - self.y) ** 2) ** 0.5
        
        if distance > self.distance_threshold:

            angle_to_target = atan2(self.TARGETS[1][self.flag] - self.y, self.TARGETS[0][self.flag] - self.x)
            angle_diff = angle_to_target - self.z
            

            angle_diff = (angle_diff + pi) % (2 * pi) - pi
            
            
            msg.linear.x = min(1.0, distance)  
            msg.angular.z = angle_diff * 2.0   
            
        else:
            if self.flag != len(self.TARGETS[0]):
                self.flag += 1
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
    def move_turtle(self):
        self.make_targets()
        self.move_targets()
        msg = Twist()
        if self.flag == 3:

            distance = ((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2) ** 0.5
        
            if distance > self.distance_threshold:

                angle_to_target = atan2(self.target_y - self.y, self.target_x - self.x)
                angle_diff = angle_to_target - self.z
            

                angle_diff = (angle_diff + pi) % (2 * pi) - pi
            
            
                msg.linear.x = min(1.0, distance)  
                msg.angular.z = angle_diff * 2.0   
            
            elif self.z > self.target_theta:
                angle_diff = self.target_theta - self.z
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

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

def main():
    rclpy.init()
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


