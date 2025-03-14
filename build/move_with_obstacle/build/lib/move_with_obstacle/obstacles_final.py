import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose  # Сообщение с текущей позицией черепахи
from custom_msgs.msg import Pose2D  # Ваше пользовательское сообщение
from math import atan2, sqrt, pi, radians
import heapq

class TurtlePlanner(Node):
    def __init__(self):
        super().__init__('turtle_planner')

        # Препятствия (последовательность точек)
        self.obstacles = [
            (2.0, 5.0),  # Точка 1
            (2.5, 7.5),  # Точка 2
            (4.0, 8.0)   # Точка 3
        ]

        # Текущая позиция черепахи
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Целевая точка
        self.target_x = None
        self.target_y = None
        self.target_theta = None  # Угол, на который нужно повернуться (в радианах)

        # Траектория движения
        self.trajectory = []

        # Подписка на топик с целевой точкой
        self.subscription = self.create_subscription(
            Pose2D,  # Тип сообщения
            '/targets',  # Имя топика
            self.target_pose_callback,  # Функция обработки сообщений
            10  # Размер очереди
        )

        # Подписка на топик с текущей позицией черепахи
        self.pose_subscription = self.create_subscription(
            Pose,  # Тип сообщения
            '/turtle1/pose',  # Имя топика
            self.pose_callback,  # Функция обработки сообщений
            10  # Размер очереди
        )

        # Публикация команд движения
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Таймер для управления движением
        self.timer = self.create_timer(0.1, self.move_turtle)

    def target_pose_callback(self, msg):
        # Обработка нового сообщения с целевой точкой
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = radians(msg.theta)  # Конвертируем градусы в радианы
        self.get_logger().info(f'Получена целевая точка: X={self.target_x}, Y={self.target_y}, Theta={self.target_theta}')

        # Планирование траектории
        self.plan_trajectory()

    def pose_callback(self, msg):
        # Обновление текущей позиции черепахи
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
        self.get_logger().info(f'Текущая позиция: X={self.current_x}, Y={self.current_y}, Theta={self.current_theta}')

    def plan_trajectory(self):
        # Очищаем старую траекторию
        self.trajectory = []

        # Используем A* для поиска пути
        start = (self.current_x, self.current_y)
        goal = (self.target_x, self.target_y)
        path = self.a_star(start, goal)

        if path:
            self.trajectory = path
            self.get_logger().info(f'Найден путь: {path}')
        else:
            # Попробуем обойти препятствия
            self.get_logger().warn('Цель заблокирована. Пытаемся обойти препятствия...')
            self.trajectory = self.find_path_around_obstacles()
            if self.trajectory:
                self.get_logger().info(f'Обходной путь: {self.trajectory}')
            else:
                self.get_logger().error('Цель недостижима!')
                self.trajectory = []

    def a_star(self, start, goal):
        # Реализация алгоритма A*
        def heuristic(a, b):
            return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if self.is_close(current, goal):
                # Восстановление пути
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # Путь не найден

    def is_close(self, point, target, threshold=0.1):
        # Проверка, близка ли точка к цели
        return sqrt((point[0] - target[0]) ** 2 + (point[1] - target[1]) ** 2) < threshold

    def get_neighbors(self, point):
        # Возвращает соседние точки (с учетом препятствий)
        neighbors = []
        for dx, dy in [(-0.5, 0), (0.5, 0), (0, -0.5), (0, 0.5), (-0.5, -0.5), (-0.5, 0.5), (0.5, -0.5), (0.5, 0.5)]:
            neighbor = (point[0] + dx, point[1] + dy)
            if not self.check_collision(point, neighbor):
                neighbors.append(neighbor)
        return neighbors

    def check_collision(self, p1, p2):
        # Проверка пересечения отрезка (p1, p2) с препятствиями
        for i in range(len(self.obstacles) - 1):
            obstacle_start = self.obstacles[i]
            obstacle_end = self.obstacles[i + 1]
            if self.segments_intersect(p1, p2, obstacle_start, obstacle_end):
                return True
        return False

    def segments_intersect(self, p1, p2, p3, p4):
        # Проверка пересечения двух отрезков
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) - (B[1] - A[1]) * (C[0] - A[0])

        A, B, C, D = p1, p2, p3, p4
        return (ccw(A, C, D) != ccw(B, C, D)) and (ccw(A, B, C) != ccw(A, B, D))

    def find_path_around_obstacles(self):
        # Простой обход препятствий: двигаться в обход первой точки препятствия
        return [
            (self.obstacles[0][0] - 1.0, self.obstacles[0][1] - 1.0),  # Обход точки A
            (self.target_x, self.target_y)  # Целевая точка
        ]

    def move_turtle(self):
        if not self.trajectory:
            return

        target_x, target_y = self.trajectory[0]

        # Вычисляем расстояние до цели
        distance = sqrt((target_x - self.current_x) ** 2 + (target_y - self.current_y) ** 2)

        if distance > 0.05:  # Порог достижения цели
            # Вычисляем угол к цели
            angle_to_target = atan2(target_y - self.current_y, target_x - self.current_x)
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
            # Удаляем достигнутую точку из траектории
            self.trajectory.pop(0)

            # Если это последняя точка, поворачиваем на угол theta
            if not self.trajectory:
                self.rotate_to_target_angle()

    def rotate_to_target_angle(self):
        # Поворачиваем черепаху на целевой угол theta
        angle_diff = self.target_theta - self.current_theta
        angle_diff = (angle_diff + pi) % (2 * pi) - pi  # Нормализация угла

        msg = Twist()
        msg.angular.z = angle_diff * 2.0  # Угловая скорость
        self.publisher.publish(msg)

        # Ждем, пока черепаха повернется
        while abs(angle_diff) > 0.1:
            angle_diff = self.target_theta - self.current_theta
            angle_diff = (angle_diff + pi) % (2 * pi) - pi
            rclpy.spin_once(self)

        self.get_logger().info('Черепаха повернута на целевой угол!')

def main(args=None):
    rclpy.init(args=args)
    turtle_planner = TurtlePlanner()
    rclpy.spin(turtle_planner)
    turtle_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
