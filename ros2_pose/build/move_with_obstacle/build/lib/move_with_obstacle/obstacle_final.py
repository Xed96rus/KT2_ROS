import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.msg import Pose2D  # Импортируем ваше пользовательское сообщение
from math import atan2, sqrt, pi

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')  # Имя ноды

        # Препятствия (цели для объезда)
        self.obstacles = [
            (2.0, 2.0, 0.0),  # (x, y, theta)
            (5.0, 5.0, 0.0),
            (8.0, 8.0, 0.0)
        ]
        self.current_target_index = 0  # Индекс текущей цели

        # Конечная цель (будет задана через топик)
        self.final_target_x = None
        self.final_target_y = None
        self.final_target_theta = None

        # Текущая позиция черепахи
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Подписка на топик с конечной целью
        self.subscription = self.create_subscription(
            Pose2D,  # Тип сообщения
            '/targets',  # Имя топика
            self.final_target_callback,  # Функция обработки сообщений
            10  # Размер очереди
        )
        self.subscription  # Предотвращаем предупреждение о неиспользуемой переменной

        # Публикация команд движения
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Таймер для управления движением
        self.timer = self.create_timer(0.1, self.move_turtle)

    def final_target_callback(self, msg):
        # Обработка нового сообщения с конечной целью
        self.final_target_x = msg.x
        self.final_target_y = msg.y
        self.final_target_theta = msg.theta
        self.get_logger().info(f'Получена конечная цель: X={self.final_target_x}, Y={self.final_target_y}, Theta={self.final_target_theta}')

        # Если конечная цель задана, начинаем движение
        if self.final_target_x is not None and self.final_target_y is not None:
            self.set_next_target()

    def set_next_target(self):
        # Устанавливаем следующую цель
        if self.current_target_index < len(self.obstacles):
            # Двигаемся к препятствию
            self.target_x, self.target_y, self.target_theta = self.obstacles[self.current_target_index]
            self.get_logger().info(f'Движение к препятствию {self.current_target_index + 1}: X={self.target_x}, Y={self.target_y}')
        else:
            # Двигаемся к конечной цели
            self.target_x = self.final_target_x
            self.target_y = self.final_target_y
            self.target_theta = self.final_target_theta
            self.get_logger().info(f'Движение к конечной цели: X={self.target_x}, Y={self.target_y}')

    def move_turtle(self):
        if self.target_x is None or self.target_y is None:
            # Если цель не задана, ждем
            self.get_logger().info('Ожидание цели...')
            return

        # Вычисляем расстояние до цели
        distance = sqrt((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)

        if distance > 0.1:  # Порог достижения цели
            # Вычисляем угол к цели
            angle_to_target = atan2(self.target_y - self.current_y, self.target_x - self.current_x)
            angle_diff = angle_to_target - self.current_theta

            # Нормализация угла
            angle_diff = (angle_diff + pi) % (2 * pi) - pi

            # Создаем сообщение для управления движением
            msg = Twist()
            msg.linear.x = min(1.0, distance)  # Линейная скорость
            msg.angular.z = angle_diff * 2.0  # Угловая скорость

            # Публикуем сообщение
            self.publisher.publish(msg)
        else:
            # Останавливаем черепаху, если цель достигнута
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)

            if self.current_target_index < len(self.obstacles):
                # Поворачиваем черепаху на угол для следующей цели
                if self.current_target_index < len(self.obstacles) - 1:
                    next_target_x, next_target_y, _ = self.obstacles[self.current_target_index + 1]
                    angle_to_next_target = atan2(next_target_y - self.current_y, next_target_x - self.current_x)
                    self.rotate_to_angle(angle_to_next_target)
                else:
                    # Объезд последнего препятствия с другой стороны
                    self.avoid_last_obstacle()

                # Переходим к следующей цели
                self.current_target_index += 1
                self.set_next_target()
            else:
                # Достигнута конечная цель
                self.get_logger().info('Конечная цель достигнута!')
                self.target_x = None
                self.target_y = None

    def rotate_to_angle(self, target_angle):
        # Поворачиваем черепаху на заданный угол
        angle_diff = target_angle - self.current_theta
        angle_diff = (angle_diff + pi) % (2 * pi) - pi  # Нормализация угла

        msg = Twist()
        msg.angular.z = angle_diff * 2.0  # Угловая скорость
        self.publisher.publish(msg)

        # Ждем, пока черепаха повернется
        while abs(angle_diff) > 0.1:
            angle_diff = target_angle - self.current_theta
            angle_diff = (angle_diff + pi) % (2 * pi) - pi
            rclpy.spin_once(self)

    def avoid_last_obstacle(self):
        # Объезд последнего препятствия с другой стороны
        last_obstacle_x, last_obstacle_y, _ = self.obstacles[-1]
        avoid_x = last_obstacle_x + 1.0  # Смещаем целевую точку в сторону
        avoid_y = last_obstacle_y + 1.0

        # Двигаемся к смещенной точке
        self.target_x = avoid_x
        self.target_y = avoid_y
        self.get_logger().info(f'Объезд последнего препятствия: X={avoid_x}, Y={avoid_y}')

    def update_current_pose(self, x, y, theta):
        # Обновляем текущую позицию черепахи
        self.current_x = x
        self.current_y = y
        self.current_theta = theta

def main(args=None):
    rclpy.init(args=args)  # Инициализация ROS2
    turtle_controller = TurtleController()  # Создаем экземпляр ноды

    # Запускаем ноду
    rclpy.spin(turtle_controller)

    # Уничтожаем ноду при завершении
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
