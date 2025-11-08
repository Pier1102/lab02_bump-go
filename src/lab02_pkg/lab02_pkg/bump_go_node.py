import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
import math


class BumpGoNode(Node):
    def __init__(self):
        super().__init__('bump_go_node')
        self.get_logger().info('Bump&Go Node started.')

        # === PARAMETRI ===
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 1.2)
        self.declare_parameter('control_loop_frequency', 10.0)
        self.declare_parameter('front_threshold', 0.45)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.frequency = self.get_parameter('control_loop_frequency').value
        self.front_threshold = self.get_parameter('front_threshold').value

        # === PUB / SUB ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # === LOOP ===
        self.timer = self.create_timer(1.0 / self.frequency, self.control_loop)

        # === STATO ===
        self.yaw = 0.0
        self.is_front_clear = True
        self.turn_left = True
        self.last_turn_left = True  # memoria direzione precedente


    # --- CALLBACK ODOM ---
    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat)


    # --- CALLBACK LIDAR ---
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max)

        # Calcola vettore degli angoli effettivi
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Normalizza nel range [-pi, pi]
        angles = np.arctan2(np.sin(angles), np.cos(angles))

        # === SELEZIONE SETTORI (simmetrici e compatibili con lidar 270°) ===
        fov = np.deg2rad(20)  # ampiezza del settore
        mask_front = np.abs(angles) < fov
        mask_left  = np.abs(angles - np.pi/2) < fov
        mask_right = np.abs(angles + np.pi/2) < fov

        # Estrai le distanze per ciascun settore
        front = ranges[mask_front]
        left  = ranges[mask_left]
        right = ranges[mask_right]

        # === FILTRAGGIO DI SICUREZZA ===
        front_min = np.min(front) if front.size > 0 else msg.range_max
        left_avg  = np.mean(left)  if left.size  > 0 else msg.range_max
        right_avg = np.mean(right) if right.size > 0 else msg.range_max

        # === LOGICA CON HYSTERESIS ===
        self.is_front_clear = front_min > self.front_threshold

        if not self.is_front_clear:
            # Mantieni la direzione scelta finché non ti liberi
            self.turn_left = self.last_turn_left
        else:
            # Aggiorna la direzione preferita solo quando la strada è libera
            self.last_turn_left = left_avg > right_avg
            self.turn_left = self.last_turn_left


    # --- CONTROL LOOP ---
    def control_loop(self):
        cmd = Twist()

        if self.is_front_clear:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('Avanti')
        else:
            cmd.linear.x = 0.0
            if self.turn_left:
                cmd.angular.z = self.angular_speed
                self.get_logger().info('Ostacolo: ruoto a SINISTRA')
            else:
                cmd.angular.z = -self.angular_speed
                self.get_logger().info('Ostacolo: ruoto a DESTRA')

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    bumpgo = BumpGoNode()
    rclpy.spin(bumpgo)
    bumpgo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()