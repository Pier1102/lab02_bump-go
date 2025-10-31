import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np


class BumpGoNode(Node):
    def __init__(self):
        super().__init__('bump_go_node')
        self.get_logger().info('Bump Go Node has been started.')

        # === Parametri ===

        self.declare_parameter('linear_speed', 0.20)  
        self.declare_parameter('angular_speed', 1.50)  
        self.declare_parameter('control_loop_frequency', 10.0) 
        self.declare_parameter('front_threshold', 0.40)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.frequency = self.get_parameter('control_loop_frequency').value
        self.front_threshold = self.get_parameter('front_threshold').value
        #self.get_logger().info(f'Parameters set: linear_speed={self.linear_speed},'f'angular_speed={self.angular_speed}, 'f'control_loop_frequency={self.frequency}')

        # === Topics ===

        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # === Loop di controllo ===
        self.timer = self.create_timer(1.0 / self.frequency, self.control_loop)

        # === Variabili di stato 

        self.yaw = 0.0 # Orientamento del robot nel piano
        self.is_front_clear = True
        self.turn_left = True

        self.get_logger().info('Bump_go_node started.')

    # === Odometry callback ===

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat) #self.yaw viene aggiornato ad ogni nuovo messaggio di odometria (/odom), convertendo il quaternione in angoli di Eulero
        
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max)

        # === Calcolo degli angoli ===
        angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))  # normalizza tra [-pi, pi]

        # === Settore frontale ===
        front = np.min(ranges[(angles < np.deg2rad(15)) & (angles > np.deg2rad(-15))])
        self.is_front_clear = front > self.front_threshold

        # === Settori laterali ===
        left_clear = np.mean(ranges[(angles > np.deg2rad(50)) & (angles < np.deg2rad(110))])
        right_clear = np.mean(ranges[(angles > np.deg2rad(-110)) & (angles < np.deg2rad(-50))])

        # === Decisione direzione ===
        self.turn_left = left_clear > right_clear

    # === Control loop ===
    def control_loop(self):
        cmd = Twist()
        if self.is_front_clear:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('Avanti')
        else:
            cmd.linear.x = 0.0
              # Gira verso il lato più libero
            if self.turn_left:
                cmd.angular.z = self.angular_speed
                self.get_logger().info('Ostacolo: ruoto a SINISTRA')
            else:
                cmd.angular.z = -self.angular_speed
                self.get_logger().info('Ostacolo: ruoto a DESTRA')
                
        self.cmd_publisher.publish(cmd)
        
def main(args=None):
    rclpy.init(args=args)
    bump_go = BumpGoNode()
    rclpy.spin(bump_go)
    bump_go.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()