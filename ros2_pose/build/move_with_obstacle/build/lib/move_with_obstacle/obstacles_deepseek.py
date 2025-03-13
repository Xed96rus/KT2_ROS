import time
import rclpy
from cmath import pi
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.msg import Pose2D
from turtlesim.msg import Pose
from math import atan2, radians, sqrt

class TurtleController(Node):
    def __init__(self):
        super().__init__('cherepavel')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.msg_subscriber = self.create_subscription(Pose2D, 'targets', self.listener_callback, 10)
        self.timer = self.create_timer(0.1, self.move_turtle)

        # Координаты препятствий [X, Y]
        self.coord_obs = [[2.0, 2.5, 4.0], [5.0, 7.5, 8.0]]
        self.TARGETS = [[0.0] * len(self.coord_obs[0]), [0.0] * len(self.coord_obs[0])]  # Инициализация целей
        self.x = 0.0  # Текущая позиция X
        self.y = 0.0  # Текущая позиция Y
        self.z = 0.0  # Текущий угол (theta)
        self.target_x = None  # Конечная цель X
        self.target_y = None  # Конечная цель Y
        self.target_theta = None  # Конечный угол
        self.flag = 0  # Индекс текущей цели
        self.distance_threshold = 0.03  # Порог достижения цели
        self.make_targets()  # Инициализация целей

    def make_targets(self):
        # Корректируем координаты целей для объезда препятствий
        for i in range(len(self.coord_obs[0])):
            for j in range(2):
                if self.coord_obs[j][i] > 5.5:
                    self.TARGETS[j][i] = self.coord_obs[j][i] - 0.5
                elif self.coord_obs[j][i] < 5.5:
                    self.TARGETS[j][i] = self.coord_obs[j][i] + 0.5
        print("Цели созданы:", self.TARGETS)

    def move_targets(self):
        msg = Twist()

        # Текущая цель
        current_target_x = self.TARGETS[0][self.flag]
        current_target_y = self.TARGETS[1][self.flag]

        # Расстояние до текущей цели
        distance = sqrt((current_target_x - self.x) ** 2 + (current_target_y - self.y) ** 2)

        if distance > self.distance_threshold:
            # Угол к текущей цели
            angle_to_target = atan2(current_target_y - self.y, current_target_x - self.x)
            angle_diff = angle_to_target - self.z
            angle_diff = (angle_diff + pi) % (2 * pi) - pi  # Нормализация угла

            # Управление скоростью
            msg.linear.x = min(1.0, distance)
            msg.angular.z = angle_diff * 2.0
        else:
            # Переход к следующей цели
            if self.flag < len(self.TARGETS[0]) - 1:
                self.flag += 1
                print(f"Переход к цели {self.flag}")
            else:
                self.flag += 1
                print("Все цели достигнуты. Движение к конечной точке.")
                return

        self.publisher_.publish(msg)

    def move_to_final_target(self):
        msg = Twist()

        # Расстояние до конечной цели
        distance = sqrt((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2)
        angle_err = self.z - self.target_theta
        if distance > self.distance_threshold:
            # Угол к конечной цели
            angle_to_target = atan2(self.target_y - self.y, self.target_x - self.x)
            angle_diff = angle_to_target - self.z
            angle_diff = (angle_diff + pi) % (2 * pi) - pi  # Нормализация угла

            # Управление скоростью
            msg.linear.x = min(1.0, distance)
            msg.angular.z = angle_diff * 2.0
        elif angle_err > self.distance_threshold or angle_err < -self.distance_threshold:
            angle_diff = self.target_theta - self.z
            msg.angular.z = angle_diff * 2.0
        else:
            # Остановка
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            print("Конечная цель достигнута!")

        self.publisher_.publish(msg)

    def move_turtle(self):
        if self.target_x is None or self.target_y is None or self.target_theta is None:
            self.get_logger().info("Ожидание цели из топика /targets")
            return
        if self.flag < len(self.TARGETS[0]):
            self.move_targets()
        else:
            self.move_to_final_target()

    def pose_callback(self, pose):
        # Обновление текущей позиции
        self.x = pose.x
        self.y = pose.y
        self.z = pose.theta
       # print(f"Текущая позиция: X={self.x:.2f}, Y={self.y:.2f}, Theta={self.z:.2f}")

    def listener_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = radians(msg.theta)
        self.get_logger().info(f'Получено сообщение: X={self.target_x}, Y={self.target_y}, Theta={self.target_theta}')

def main():
    rclpy.init()
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
